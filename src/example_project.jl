using LinearAlgebra, StaticArrays

struct MyLocalizationType
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end
"""
    lerp(a, b, t)

Linearly interpolate between `a` and `b` by fraction `t` (where `t=0` gives `a`, `t=1` gives `b`).
"""
function lerp(a, b, t)
    return a + (b - a) * t
end


function angle_diff(a, b)
    d = a - b
    return mod(d + π, 2π) - π
end

# Helper: Compute the 2D distance from vehicle position to the endpoint of a segment.
# Here, we use the endpoint of lane-boundary 1 (pt_b) as the segment’s endpoint.
function segment_end_distance(seg::VehicleSim.RoadSegment, pos::Vector{Float64})
    endpoint = collect(seg.lane_boundaries[1].pt_b)[1:2]
    return norm(pos - endpoint)
end

# Helper: Compute the parking target for a loading zone segment.
function parking_target(seg::VehicleSim.RoadSegment)
    # Convert the endpoints of lane-boundaries 2 and 3 to 2D coordinates.
    pt2 = collect(seg.lane_boundaries[2].pt_b)[1:2]
    pt3 = collect(seg.lane_boundaries[3].pt_b)[1:2]
    return 0.5 .* (pt2 .+ pt3)
end

has_stop_sign(seg::VehicleSim.RoadSegment) = any(t -> t == VehicleSim.RoadSegment.LaneTypes.stop_sign, seg.lane_types)


function process_gt(gt_channel, shutdown_channel, target_segment_channel, road_map::Dict{Int,VehicleSim.RoadSegment}, socket, ego_vehicle_id_channel)
    # --- Initial Routing (hard‐coded route) ---
    while !isready(gt_channel)
        sleep(0.01)
    end

    while fetch(ego_vehicle_id_channel) == 0
        sleep(0.01)
    end
    ego_vehicle_id = fetch(ego_vehicle_id_channel)
    println("process_gt: ego_vehicle_id = $ego_vehicle_id")
    ego_vehicle_id = fetch(ego_vehicle_id_channel)
    while true
        m = take!(gt_channel)
        if m.vehicle_id == ego_vehicle_id
            global vehicle_pos     = collect(m.position[1:2])
            global vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(m.orientation))
            break
        end
    end

    # get target and plan route
    target_seg_id = fetch(target_segment_channel)
    start_seg_id  = find_nearest_segment(vehicle_pos, road_map)
    route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
    if isempty(route)
        println("No valid route found from segment $start_seg_id to $target_seg_id. Aborting route execution.")
        return
    end
    println("Computed route: ", route)

    current_index = 1
    default_speed   = 5.0
    threshold       = 7.0   # segment‐end transition
    stop_dist       = 20.0  # frontal blocking distance
    clear_dist      = 15.0  # beyond which we consider the obstacle gone
    lateral_thresh  = 2.0   # half‐width of blocking “lane”

    # buffer for latest GT of every vehicle
    latest_gt = Dict{Int,GroundTruthMeasurement}()

    # --- Main Loop: Follow the Precomputed Route ---
    while true
        # Shutdown check
        if isready(shutdown_channel) && fetch(shutdown_channel)
            break
        end
        # 1) drain gt_channel into buffer
        while isready(gt_channel)
            m = take!(gt_channel)
            latest_gt[m.vehicle_id] = m
        end
        # 1) Make sure we’ve got a valid ego ID (non‑zero)
        if ego_vehicle_id == 0
            @warn "process_gt: still waiting on a real ego_vehicle_id…"
            sleep(0.01)
            continue
        end

        # 2) Wait until latest_gt actually contains our vehicle’s measurement
        if !haskey(latest_gt, ego_vehicle_id)
            @warn "process_gt: no ground‐truth for ego ID=$ego_vehicle_id yet, retrying…"
            sleep(0.005)
            continue
        end

        # 3) Now it’s safe to extract
        self_meas   = latest_gt[ego_vehicle_id]
        vehicle_pos = collect(self_meas.position[1:2])
        vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(self_meas.orientation))

            # --- Vehicle‑Ahead Check (2D bbox) ---
        B = 0.5                              # safety buffer [m]
        eff_lateral = lateral_thresh + B
        eff_forward = stop_dist     + B

        blocked = false
        # approximate each car as a circle of radius R, plus a safety buffer B
        R = 1.0            # ≈ half the width of your vehicle [m]
        B = 0.5            # extra margin [m]
        blocked = false
        for (vid, meas) in latest_gt
            vid == ego_vehicle_id && continue
            # 1) compute other‑vehicle center in ego frame
            dx, dy = meas.position[1] - vehicle_pos[1], meas.position[2] - vehicle_pos[2]
            rel_x =  cos(vehicle_heading)*dx + sin(vehicle_heading)*dy
            rel_y = -sin(vehicle_heading)*dx + cos(vehicle_heading)*dy

            # 2) inflate front‐detection corridor by R+B
            eff_lateral = lateral_thresh + R + B
            eff_forward = stop_dist      + R + B

            # 3) check: any part of that circle sits inside your stopping corridor
            if rel_x > 0 && abs(rel_y) < eff_lateral && rel_x < eff_forward
                blocked = true
                break
            end
        end

        if blocked
            # println("Vehicle detected ahead")
            # println("Relative difference (X-axis): ", rel_x)
            # println("Relative difference (Y-axis): ", rel_y)
            # println("Stopping vehicle")
            serialize(socket, (0.0, 0.0, true))
            continue
        end


        # 3) segment‐following logic
        current_seg_id = route[current_index]
        current_seg    = road_map[current_seg_id]

        if current_index < length(route)
            # steering toward look‐ahead point
            lb1_end = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
            lb2_end = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
            lookahead = 0.5 .* (lb1_end .+ lb2_end)
            desired_heading = atan(lookahead[2] - vehicle_pos[2], lookahead[1] - vehicle_pos[1])
            steering = clamp(angle_diff(desired_heading, vehicle_heading), -1, 1)
            serialize(socket, (steering, default_speed, true))

            # check for stop signs
            dist_end = segment_end_distance(current_seg, vehicle_pos)
            if 5 < dist_end < 13 && current_seg.lane_types[1] == VehicleSim.stop_sign
                println("STOP at stop sign")
                serialize(socket, (0.0, 0.0, true)); sleep(3.0)
                current_index += 1
                serialize(socket, (0.0, 7.0, true))
                continue
            end

            # advance segment
            if dist_end < threshold
                current_index += 1
                continue
            end

        else
            # last segment → parking mode
            A = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
            B = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
            C = collect(current_seg.lane_boundaries[3].pt_a)[1:2]
            D = collect(current_seg.lane_boundaries[3].pt_b)[1:2]
            center = 0.25 .* (A .+ B .+ C .+ D)
            d2c = norm(vehicle_pos - center)
            if d2c < 4
                println("Parking complete — waiting for new target…")
                serialize(socket, (0.0, 0.0, true))
        
                # wait here until target_segment_channel gives us a genuinely new goal
                while true
                    if isready(target_segment_channel)
                        new_t = take!(target_segment_channel)
                        put!(target_segment_channel, new_t)
                        if new_t != target_seg_id
                            println(" Got new target $new_t — replanning route")
                            start_seg_id = target_seg_id
                            target_seg_id = new_t
                            route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
                            current_index = 1
                            println("New route: ", route)
                            break    # exit this wait‐loop and resume driving
                        end
                    end
                    sleep(0.1)
                end
                # println("At loading‐zone center. Stopping.")
                # serialize(socket, (0.0, 0.0, true))
                # break
            else
                # drive into center
                desired_heading = atan(center[2] - vehicle_pos[2], center[1] - vehicle_pos[1])
                steering = angle_diff(desired_heading, vehicle_heading)
                speed = default_speed * min(1.0, d2c/5.0)
                serialize(socket, (steering, speed, true))
                continue
            end
        end
    end
end


function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)
    # --- Initialization: Wait for the first GPS measurement ---
    println("Waiting for initial GPS measurement for EKF initialization...")
    while !isready(gps_channel)
        if isready(shutdown_channel) && fetch(shutdown_channel)
            return  # Exit early if shutdown is signalled
        end
        sleep(0.01)
    end
    initial_gps = take!(gps_channel)
    println("Initial GPS measurement: ", initial_gps)
    
    # Initialize state vector (13-dimensional)
    x0 = zeros(13)
    # Use raw GPS coordinates directly
    x0[1] = initial_gps.lat   # Latitude as x position
    x0[2] = initial_gps.long  # Longitude as y position
    x0[3] = 0.0               # Altitude (z position)
    
    # Initialize orientation based on GPS heading
    # Convert heading to quaternion (assuming heading is in radians)
    x0[4] = cos(initial_gps.heading/2)  # Quaternion w component
    x0[5] = 0.0                         # Quaternion x component
    x0[6] = 0.0                         # Quaternion y component
    x0[7] = sin(initial_gps.heading/2)  # Quaternion z component
    
    # Initialize velocities to zero
    x0[8:13] .= 0.0
    
    # Initial covariance
    P0 = 0.1 * Matrix(1.0I, 13, 13)
    
    # Create initial EKF states
    ekf_gps = SkeletonsAVStack.Localization.EKFState(x0, P0)
    ekf_imu = SkeletonsAVStack.Localization.EKFState(x0, P0)
    
    # Fuse the two branches and publish the initial state
    fused_state = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
    put!(localization_state_channel, fused_state.x)
    println("Initial fused state: ", fused_state.x)
    
    t_last = time()
    
    # Main loop: update continuously
    while true
        if isready(shutdown_channel) && fetch(shutdown_channel)
            println("Shutdown signal received. Exiting localization loop.")
            break
        end

        # Collect fresh GPS and IMU measurements
        fresh_gps_meas = []
        while isready(gps_channel)
            push!(fresh_gps_meas, take!(gps_channel))
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            push!(fresh_imu_meas, take!(imu_channel))
        end
        
        dt = time() - t_last
        t_last = time()
        
        # Prediction step for both EKF branches
        ekf_gps = SkeletonsAVStack.Localization.ekf_predict(ekf_gps, dt)
        ekf_imu = SkeletonsAVStack.Localization.ekf_predict(ekf_imu, dt)
        
        # Process all fresh GPS measurements
        for gps in fresh_gps_meas
            # Use raw GPS coordinates directly
            z_gps = [gps.lat, gps.long, gps.heading]
            # Measurement noise - adjust these values based on your GPS accuracy
            R_gps = Diagonal([1.0, 1.0, 1.0])  # [position variance, position variance, heading variance]
            ekf_gps = SkeletonsAVStack.Localization.ekf_update_gps(ekf_gps, z_gps, R_gps)
        end
        
        # Process all fresh IMU measurements
        for imu in fresh_imu_meas
            z_imu = vcat(Vector(imu.linear_vel), Vector(imu.angular_vel))
            R_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
            ekf_imu = SkeletonsAVStack.Localization.ekf_update_imu(ekf_imu, z_imu, R_imu)
        end
        
        # Fuse the two branches
        fused = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
        
        # Publish the new state
        if !isempty(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, fused.x)
        
        sleep(0.01)  
    end
end






# Helper function to find the nearest road segment to the vehicle position.

function find_nearest_segment(vehicle_pos::Vector{Float64}, map::Dict{Int, VehicleSim.RoadSegment})
    min_dist = Inf
    nearest_seg = nothing
    for (seg_id, seg) in map
        # Use the midpoint of the first lane boundary as a representative position.
        midpt = (seg.lane_boundaries[1].pt_a .+ seg.lane_boundaries[1].pt_b) ./ 2
        d = norm(vehicle_pos .- midpt)
        if d < min_dist
            min_dist = d
            nearest_seg = seg_id
        end
    end
    return nearest_seg
end

# Helper function to check if a road segment is curved.
function is_curved(seg::VehicleSim.RoadSegment; atol=1e-6)
    for lb in seg.lane_boundaries
        if !isapprox(lb.curvature, 0.0; atol=atol)
            return true
        end
    end
    return false
end


function decision_making(localization_state_channel, perception_state_channel, 
                        target_segment_channel, shutdown_channel, 
                        road_map::Dict{Int,VehicleSim.RoadSegment}, socket)
    # --- Initial Routing ---
    while !isready(localization_state_channel)
        sleep(0.01)
    end
    initial_loc = take!(localization_state_channel)
    vehicle_pos = initial_loc[1:2]
    vehicle_heading = VehicleSim.extract_yaw_from_quaternion(initial_loc[4:7])
    
    target_seg_id = fetch(target_segment_channel)
    start_seg_id = find_nearest_segment(vehicle_pos, road_map)
    route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
    if isempty(route)
        println("No valid route found from segment $start_seg_id to $target_seg_id.")
        return
    end
    println("Computed route: ", route)
    current_index = 1
    current_seg = road_map[route[current_index]]
    
    # --- Control Parameters ---
    default_speed = 5.0  # Default speed in m/s
    lookahead_distance = 8.0  # Lookahead distance in meters
    k_p = 0.8  # Proportional gain for steering control
    max_steering = π/4  # Maximum steering angle in radians
    
    # --- Main Loop ---
    while true
        if isready(shutdown_channel) && fetch(shutdown_channel)
            break
        end
        
        # Get latest state
        while !isready(localization_state_channel)
            sleep(0.001)
        end
        latest_loc = take!(localization_state_channel)
        vehicle_pos = latest_loc[1:2]
        vehicle_heading = VehicleSim.extract_yaw_from_quaternion(latest_loc[4:7])


        # 2) Perception‐based obstacle check
        latest_perc = Perception.TrackedObject[]
        while isready(perception_state_channel)
            latest_perc = take!(perception_state_channel)
        end

        blocked = false
        for obj in latest_perc
            # obj.position is [x_lateral, y_vertical?, z_forward] in camera frame
            x_lat, _, z_fwd = obj.position
            println("  Detected obj=$(obj.id): X=$(round(x_lat,1)) m  Z=$(round(z_fwd,1)) m")
            if z_fwd > 0 && abs(x_lat) <= LATERAL_THRESH && z_fwd <= FORWARD_THRESH
                blocked = true
                break
            end
        end

        if blocked
            println("Decision-making: obstacle detected ahead (perception) → stopping.")
            serialize(socket, (0.0, 0.0, true))
            sleep(0.1)
            continue
        end
        
        current_seg_id = route[current_index]
        current_seg = road_map[current_seg_id]
        
        if current_index < length(route)
            
            lb1_end = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
            lb2_end = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
            lookahead_point = 0.5 .* (lb1_end .+ lb2_end)
            desired_heading = atan(lookahead_point[2] - vehicle_pos[2], 
                                  lookahead_point[1] - vehicle_pos[1])
            heading_error = angle_diff(desired_heading, vehicle_heading)
            
            # Calculate steering angle based on heading error
            if is_curved(current_seg)
                curvature = current_seg.lane_boundaries[1].curvature
                curvature_term = 0.5 * curvature * default_speed
            else
                curvature_term = 0.0
            end
            
            steering_angle = k_p * heading_error + curvature_term
            steering_angle = clamp(steering_angle, -max_steering, max_steering)
            
            # Calculate distance to segment end
            dist_to_end = segment_end_distance(current_seg, vehicle_pos)
            speed_factor = min(1.0, dist_to_end / 10.0)  
            target_vel = default_speed * speed_factor
            
            # Check for stop signs
            if 5 < dist_to_end < 15 && current_seg.lane_types[1] == VehicleSim.stop_sign
                println("Approaching stop sign")
                cmd = (0.0, 0.0, true)
                serialize(socket, cmd)
                sleep(3.0)
                current_index += 1
                continue
            end
            
            # Check for segment transition
            if dist_to_end < 10.0
                current_index += 1
                println("Transitioning to segment ", route[current_index])
                continue
            end
            
            cmd = (steering_angle, target_vel, true)
            serialize(socket, cmd)
            
        else
            # Parking at target
            A = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
            B = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
            C = collect(current_seg.lane_boundaries[3].pt_a)[1:2]
            D = collect(current_seg.lane_boundaries[3].pt_b)[1:2]
            center = 0.25 .* (A .+ B .+ C .+ D)
            dist_to_center = norm(vehicle_pos - center)
            
            if dist_to_center < 5
                println("Parking complete — waiting for new target…")
                serialize(socket, (0.0, 0.0, true))
                while true
                    if isready(target_segment_channel)
                        new_t = take!(target_segment_channel)
                        put!(target_segment_channel, new_t)
                        if new_t != target_seg_id
                            println(" Got new target $new_t — replanning route")
                            start_seg_id = target_seg_id
                            target_seg_id = new_t
                            route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
                            current_index = 1
                            println("New route: ", route)
                            break    # exit this wait‐loop and resume driving
                        end
                    end
                    sleep(0.1)
                end
            else
                desired_heading = atan(center[2] - vehicle_pos[2], center[1] - vehicle_pos[1])
                steering_angle = angle_diff(desired_heading, vehicle_heading)
                target_vel = default_speed * min(1.0, dist_to_center / 5.0)
                cmd = (steering_angle, target_vel, true)
                serialize(socket, cmd)
            end
        end
    end
end

# Updated Perception Function
function perception(cam_meas_channel,
                    perception_state_channel,
                    shutdown_channel)
    while true
        # Shutdown check
        if isready(shutdown_channel) && fetch(shutdown_channel)
            println("Perception: shutting down.")
            break
        end

        # 1) Drain all camera measurements
        fresh = CameraMeasurement[]
        while isready(cam_meas_channel)
            push!(fresh, take!(cam_meas_channel))
        end

        # If no new frames, wait for next
        if isempty(fresh)
            sleep(0.05)
            continue
        end

        # 2) Log each frame and its bounding boxes
        for cam in fresh
            @info "Perception: camera=$(cam.camera_id) | boxes=$(length(cam.bounding_boxes))"
        end

        # 3) Process and accumulate tracked objects
        all_tracked = SkeletonsAVStack.Perception.TrackedObject[]
        for cam in fresh
            tracked = SkeletonsAVStack.Perception.process_camera_measurement(cam)
            append!(all_tracked, tracked)
        end

        # 4) Publish the new perception state (overwrite old)
        if !isempty(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, all_tracked)

        # 5) Debug print
        if isempty(all_tracked)
            println("Perception: No objects detected.")
        else
            println("Perception: Detected objects:")
            for obj in all_tracked
                println("  ID=$(obj.id) | pos=$(obj.position) | bbox=$(obj.bbox)")
            end
        end

        sleep(0.05)
    end
end

# Helper function to check if a channel is full.
function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444; use_gt=false)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    # localization_state_channel = Channel{MyLocalizationType}(1)
    localization_state_channel = Channel{Vector{Float64}}(1)
    # perception_state_channel = Channel{MyPerceptionType}(1)
    perception_state_channel = Channel{Vector{Float64}}(1)
    target_segment_channel = Channel{Int}(1)
    ego_vehicle_id_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)
    put!(ego_vehicle_id_channel, ego_vehicle_id)

    errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
        target_map_segment = measurement_msg.target_segment
        old_target_segment = fetch(target_segment_channel)
        if target_map_segment ≠ old_target_segment
            take!(target_segment_channel)
            put!(target_segment_channel, target_map_segment)
        end
        if fetch(ego_vehicle_id_channel) == 0
            ego_vehicle_id = measurement_msg.vehicle_id
            take!(ego_vehicle_id_channel)
            put!(ego_vehicle_id_channel, ego_vehicle_id)
        else
            ego_vehicle_id = fetch(ego_vehicle_id_channel)
        end
        println("I am Vehicle No.", ego_vehicle_id)
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement && measurement_msg.vehicle_id == ego_vehicle_id
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement && measurement_msg.vehicle_id == ego_vehicle_id
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement && measurement_msg.vehicle_id == ego_vehicle_id
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    if use_gt
        @async process_gt(gt_channel,
                      shutdown_channel, target_segment_channel, 
                      map_segments, socket, ego_vehicle_id_channel)
    else
        @async localize(gps_channel, 
                    imu_channel, 
                    localization_state_channel, 
                    shutdown_channel)

        @async perception(cam_channel,
                      perception_state_channel, 
                      shutdown_channel)

        @async decision_making(localization_state_channel, 
            perception_state_channel, 
            target_segment_channel, 
            shutdown_channel,
            map_segments, 
            socket)
    end
end

function shutdown_listener(shutdown_channel)
    info_string = 
        "***************
      CLIENT COMMANDS
      ***************
            -Make sure focus is on this terminal window. Then:
            -Press 'q' to shutdown threads. 
    "
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == 'q'
            # terminate threads
            take!(shutdown_channel)
            put!(shutdown_channel, true)
            break
        end
    end
end