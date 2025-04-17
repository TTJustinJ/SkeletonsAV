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
# For a loading zone, lane_boundaries[2] and [3] form the “waist” (梯形的腰); we take the midpoint.
function parking_target(seg::VehicleSim.RoadSegment)
    # Convert the endpoints of lane-boundaries 2 and 3 to 2D coordinates.
    pt2 = collect(seg.lane_boundaries[2].pt_b)[1:2]
    pt3 = collect(seg.lane_boundaries[3].pt_b)[1:2]
    return 0.5 .* (pt2 .+ pt3)
end

# function process_gt(gt_channel, shutdown_channel, target_segment_channel, perception_state_channel, road_map::Dict{Int,VehicleSim.RoadSegment}, socket)
#     # --- Initial Routing (compute once) ---
#     while !isready(gt_channel)
#         sleep(0.01)
#     end
#     initial_gt = take!(gt_channel)
#     vehicle_pos = collect(initial_gt.position[1:2])
#     vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(initial_gt.orientation))
    
#     target_seg_id = fetch(target_segment_channel)
#     start_seg_id = find_nearest_segment(vehicle_pos, road_map)
#     route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
#     if isempty(route)
#         println("No valid route found from segment $start_seg_id to $target_seg_id. Aborting route execution.")
#         return
#     end
#     println("Computed route: ", route)
#     current_index = 1
#     current_seg = road_map[ route[current_index] ]
    
#     threshold = 5.5           # For normal segment endpoint transition.
#     parking_threshold = 5.5   # When inside a loading zone, distance at which we consider parking complete.
#     default_speed = 8.0       # Default cruising speed.
    
#     # --- Main Loop: Follow the Precomputed Route ---
#     while true
#         if isready(shutdown_channel) && fetch(shutdown_channel)
#             break
#         end
        
#         while !isready(gt_channel)
#             sleep(0.001)
#         end
#         latest_gt = take!(gt_channel)
#         vehicle_pos = collect(latest_gt.position[1:2])
#         vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(latest_gt.orientation))
        
#         # Re-acquire the current segment from the route.
#         current_seg_id = route[current_index]
#         current_seg = road_map[ current_seg_id ]
#         # --- Normal Behavior ---
#         # Check if the vehicle is near the endpoint of the current segment.
#         if current_seg_id == target_seg_id
#             # 计算到loading zone中心的距离
#             A = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
#             B = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
#             C = collect(current_seg.lane_boundaries[3].pt_a)[1:2]
#             D = collect(current_seg.lane_boundaries[3].pt_b)[1:2]

#             center = [(A[1]+B[1]+C[1]+D[1])/4, (A[2]+B[2]+C[2]+D[2])/4]
#             println(center[1], center[2])
#             dist_to_center = norm(vehicle_pos - center)
#             if dist_to_center < parking_threshold
#                 println("Target segment reached. Stopping vehicle.")
#                 cmd = (0.0, 0.0, true)  # 停止命令
#                 serialize(socket, cmd)
#                 break
#             else
#                 # 继续向loading zone中心移动
#                 lookahead_point = center
#                 desired_heading = atan(lookahead_point[2] - vehicle_pos[2],
#                                      lookahead_point[1] - vehicle_pos[1])
#                 steering_angle = desired_heading - vehicle_heading
#                 print("steering angle", steering_angle)
#                 target_vel = default_speed * min(1.0, dist_to_center/5.0)  # 接近时减速
#                 cmd = (steering_angle, target_vel, true)
#                 serialize(socket, cmd)
#                 continue
#             end
#         else
#             # 修改segment切换逻辑 - 只有在不是最后一个segment时才切换
#             dist_to_end = segment_end_distance(current_seg, vehicle_pos)
#             if dist_to_end < threshold && current_index < length(route)
#                 println("Reached endpoint of segment ", route[current_index], " (distance = ", dist_to_end, ").")
#                 next_seg_expected = route[current_index+1]
#                 current_index += 1
#                 current_seg = road_map[next_seg_expected]
#                 println("Transitioning to child segment ", next_seg_expected)
#             end
                
#             lb1_endpoint = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
#             lb2_endpoint = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
#             lookahead_point = 0.5 .* (lb1_endpoint .+ lb2_endpoint)
#             desired_heading = atan(lookahead_point[2] - vehicle_pos[2],
#                                         lookahead_point[1] - vehicle_pos[1])
#             steering_angle = angle_diff(desired_heading, vehicle_heading)
#             target_vel = default_speed
            
#             # println("Vehicle pos: ", vehicle_pos, " | Current seg: ", route[current_index],
#             #         " | Route index: ", current_index, " | Steering angle: ", steering_angle,
#             #         " | Target vel: ", target_vel)
            
#             # Command the vehicle.
#             cmd = (steering_angle, target_vel, true)
#             serialize(socket, cmd)
#             sleep(0.01)
#         end
#     end
# end
# function process_gt(gt_channel, shutdown_channel, target_segment_channel, perception_state_channel, road_map::Dict{Int,VehicleSim.RoadSegment}, socket)
#     # --- Initial Routing (hard-coded route) ---
#     while !isready(gt_channel)
#         sleep(0.01)
#     end
#     initial_gt = take!(gt_channel)
#     vehicle_pos = collect(initial_gt.position[1:2])
#     vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(initial_gt.orientation))
    
#     target_seg_id = fetch(target_segment_channel)
#     start_seg_id = find_nearest_segment(vehicle_pos, road_map)
#     route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
#     if isempty(route)
#         println("No valid route found from segment $start_seg_id to $target_seg_id. Aborting route execution.")
#         return
#     end
#     println("Computed route: ", route)
#     println("Using hard-coded route: ", route)
#     current_index = 1

#     default_speed = 8.0

#     # --- Main Loop: Follow the Precomputed Route ---
#     while true
#         # Check for shutdown
#         if isready(shutdown_channel) && fetch(shutdown_channel)
#             break
#         end

#         # Get latest GT
#         while !isready(gt_channel)
#             sleep(0.001)
#         end
#         latest_gt = take!(gt_channel)
#         vehicle_pos = collect(latest_gt.position[1:2])
#         vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(latest_gt.orientation))

#         # Get current segment from route
#         current_seg_id = route[current_index]
#         current_seg = road_map[current_seg_id]

#         # Determine bounding box based on segment type
#         if current_index == length(route)
#             # Loading zone: use lane boundaries 2 & 3
#             A = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
#             B = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
#             C = collect(current_seg.lane_boundaries[3].pt_a)[1:2]
#             D = collect(current_seg.lane_boundaries[3].pt_b)[1:2]
#         else
#             # Normal segment: use lane boundaries 1 & 2
#             A = collect(current_seg.lane_boundaries[1].pt_a)[1:2]
#             B = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
#             C = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
#             D = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
#         end
#         min_x = minimum([A[1], B[1], C[1], D[1]])
#         max_x = maximum([A[1], B[1], C[1], D[1]])
#         min_y = minimum([A[2], B[2], C[2], D[2]])
#         max_y = maximum([A[2], B[2], C[2], D[2]])

#         # Check if vehicle is within bounding box
#         in_box = (min_x <= vehicle_pos[1] <= max_x) && (min_y <= vehicle_pos[2] <= max_y)
#         println(in_box, "position", vehicle_pos)
#         if in_box
#             if current_index == length(route)
#                 # Last segment -> parking mode
#                 center = 0.25 .* (A .+ B .+ C .+ D)
#                 dist_to_center = norm(vehicle_pos - center)
#                 if dist_to_center < 0.5
#                     println("At center of loading zone. Stopping vehicle.")
#                     cmd = (0.0, 0.0, true)
#                     serialize(socket, cmd)
#                     break
#                 else
#                     # Move toward center
#                     desired_heading = atan(center[2] - vehicle_pos[2], center[1] - vehicle_pos[1])
#                     steering_angle = desired_heading - vehicle_heading
#                     target_vel = default_speed * min(1.0, dist_to_center / 5.0)
#                     cmd = (steering_angle, target_vel, true)
#                     serialize(socket, cmd)
#                     continue
#                 end
#             else
#                 # Reached normal segment -> advance to next
#                 lb1_endpoint = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
#                 lb2_endpoint = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
#                 lookahead_point = 0.5 .* (lb1_endpoint .+ lb2_endpoint)
#                 desired_heading = atan(lookahead_point[2] - vehicle_pos[2], lookahead_point[1] - vehicle_pos[1])
#                 steering_angle = angle_diff(desired_heading, vehicle_heading)
#                 cmd = (steering_angle, default_speed, true)
#                 serialize(socket, cmd)
#                 println("Vehicle pos: ", vehicle_pos," | Steering angle: ", steering_angle,
#                     " | Target vel: ", default_speed)
#                 # Advance index
#                 println("Transitioning from segment ", current_seg_id, " to ", route[current_index+1])
#                 current_index += 1
#                 continue
#             end
#         else
#             # Not yet in segment -> maintain current command
#             # Could log or keep previous cmd
#             # Here, we simply wait for vehicle to enter box
#             sleep(0.01)
#             continue
#         end
#     end
# end

has_stop_sign(seg::VehicleSim.RoadSegment) = any(t -> t == VehicleSim.RoadSegment.LaneTypes.stop_sign, seg.lane_types)


function process_gt(gt_channel, shutdown_channel, target_segment_channel, perception_state_channel, road_map::Dict{Int,VehicleSim.RoadSegment}, socket)
    # --- Initial Routing (hard-coded route) ---
    while !isready(gt_channel)
        sleep(0.01)
    end
    initial_gt = take!(gt_channel)
    vehicle_pos = collect(initial_gt.position[1:2])
    vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(initial_gt.orientation))

    target_seg_id = fetch(target_segment_channel)
    start_seg_id = find_nearest_segment(vehicle_pos, road_map)
    route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
    if isempty(route)
        println("No valid route found from segment $start_seg_id to $target_seg_id. Aborting route execution.")
        return
    end
    println("Computed route: ", route)


    println("segment 14", road_map[14])
    current_index = 1

    default_speed = 8.5
    threshold = 7.0  # distance threshold for segment transitions

    # --- Main Loop: Follow the Precomputed Route ---
    while true
        # Shutdown check
        if isready(shutdown_channel) && fetch(shutdown_channel)
            break
        end

        # Get latest GT
        while !isready(gt_channel)
            sleep(0.001)
        end
        latest_gt = take!(gt_channel)
        vehicle_pos = collect(latest_gt.position[1:2])
        vehicle_heading = VehicleSim.extract_yaw_from_quaternion(collect(latest_gt.orientation))

        # Current segment
        current_seg_id = route[current_index]
        current_seg = road_map[current_seg_id]
        # println("current segment id: ", current_seg_id)
        if current_index < length(route)
            # Steering toward lookahead point
            lb1_end = collect(current_seg.lane_boundaries[1].pt_b)[1:2]
            lb2_end = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
            lookahead_point = 0.5 .* (lb1_end .+ lb2_end)
            # println("lookahead point: ", lookahead_point)
            desired_heading = atan(lookahead_point[2] - vehicle_pos[2], lookahead_point[1] - vehicle_pos[1])
            steering_angle = angle_diff(desired_heading, vehicle_heading)
            steering_angle = clamp(steering_angle, -1, 1)
            # println("steering_angle, ", steering_angle)
            cmd = (steering_angle, default_speed, true)
            serialize(socket, cmd)
            # --- Normal segment: use endpoint distance check ---
            dist_to_end = segment_end_distance(current_seg, vehicle_pos)
            if 5 < dist_to_end < 13 
                if current_seg.lane_types[1] == VehicleSim.stop_sign
                    println("STOP!!!!!!!!!!!!")
                    cmd = (0, 0, true)
                    serialize(socket, cmd)
                    sleep(3.0)
                    current_index +=1
                    cmd = (0, 7.0, true)
                    serialize(socket, cmd)
                    continue
                end
            end
            if dist_to_end < threshold
                # println("Reached end of segment ", current_seg_id, " (dist=", dist_to_end, ").")
                current_index += 1
                # println("Advancing to segment ", route[current_index])
             
                continue
            end
        else
            if current_index == length(route)
                # Last segment -> parking mode
                A = collect(current_seg.lane_boundaries[2].pt_a)[1:2]
                B = collect(current_seg.lane_boundaries[2].pt_b)[1:2]
                C = collect(current_seg.lane_boundaries[3].pt_a)[1:2]
                D = collect(current_seg.lane_boundaries[3].pt_b)[1:2]
                center = 0.25 .* (A .+ B .+ C .+ D)
                dist_to_center = norm(vehicle_pos - center)
                if dist_to_center < 0.5
                    println("At center of loading zone. Stopping vehicle.")
                    cmd = (0.0, 0.0, true)
                    serialize(socket, cmd)
                    break
                else
                    # Move toward center
                    desired_heading = atan(center[2] - vehicle_pos[2], center[1] - vehicle_pos[1])
                    steering_angle = angle_diff(desired_heading, vehicle_heading)
                    target_vel = default_speed * min(1.0, dist_to_center / 5.0)
                    cmd = (steering_angle, target_vel, true)
                    serialize(socket, cmd)
                    continue
                end
            end
        end
    end
end



# function localize(
#         gps_channel, 
#         imu_channel, 
#         localization_state_channel, 
#         shutdown_channel)

#     # --- Initialization ---
#     # Set up algorithm / initialize variables
#     println("Waiting for initial GPS measurement for EKF initialization...")
#     while !isready(gps_channel)
#         if isready(shutdown_channel)
#             if fetch(shutdown_channel)
#                 return  # Exit early if a shutdown has been signaled.
#             end
#         end
#         sleep(0.01)
#     end
#     # Get the first GPS measurement (assumed to be of type GPSMeasurement with fields lat, long, heading)
#     initial_gps = take!(gps_channel)
#     println("Initial GPS measurement: ", initial_gps)
    
#     # Initialize state vector (13-dimensional).
#     x0 = zeros(13)
#     # Since h_gps already returns positions in meters, use them directly.
#     x0[1] = initial_gps.lat
#     x0[2] = initial_gps.long
#     x0[3] = 0.0
#     # 在localization模块中打印原始GPS坐标和转换后的局部坐标
#     println("Raw GPS (lat,lon): ", initial_gps.lat, ", ", initial_gps.long)
#     println("Localized position (x,y): ", x0[1:2])
#     # Initialize orientation as identity quaternion (or based on heading):
#     x0[4] = 1.0; x0[5:7] .= 0.0
#     # Initialize velocity and angular velocity to zero.
#     x0[8:13] .= 0.0
#     # Initial covariance.
#     P0 = 0.1 * Matrix(1.0I, 13, 13)
    
#     # Initialize both EKF branches with the same initial state.
#     ekf_gps = SkeletonsAVStack.Localization.EKFState(x0, P0)
#     ekf_imu = SkeletonsAVStack.Localization.EKFState(x0, P0)
    
#     # Publish initial fused state.
#     fused_state = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
#     put!(localization_state_channel, fused_state.x)
#     println("Initial fused state: ", fused_state.x)
    
#     t_last = time()
    
#     # Main loop.
#     while true
#         if isready(shutdown_channel) && fetch(shutdown_channel)
#             println("Shutdown signal received. Exiting localization loop.")
#             break
#         end

#         # fetch(shutdown_channel) && break

#         # Collect fresh measurements.
#         fresh_gps_meas = []
#         while isready(gps_channel)
#             push!(fresh_gps_meas, take!(gps_channel))
#         end


#         fresh_imu_meas = []
#         while isready(imu_channel)
#             push!(fresh_imu_meas, take!(imu_channel))
#         end
        
#         dt = time() - t_last
#         t_last = time()
        
#         # Prediction step for both branches.
#         ekf_gps = SkeletonsAVStack.Localization.ekf_predict(ekf_gps, dt)
#         ekf_imu = SkeletonsAVStack.Localization.ekf_predict(ekf_imu, dt)
        
#         # Process all fresh GPS measurements (update the GPS branch).
#         for gps in fresh_gps_meas
#             # Construct measurement vector from GPSMeasurement: [lat, long, heading]
#             z_gps = [gps.lat, gps.long, gps.heading]
#             R_gps = Diagonal([1.0, 1.0, 0.1])  # Tune measurement noise as needed.
#             ekf_gps = SkeletonsAVStack.Localization.ekf_update_gps(ekf_gps, z_gps, R_gps)
#         end
        
#         # Process all fresh IMU measurements (update the IMU branch).
#         for imu in fresh_imu_meas
#             # Construct measurement vector from IMUMeasurement: 
#             # Concatenate linear_vel and angular_vel, converting from SVector to Vector.
#             z_imu = vcat(Vector(imu.linear_vel), Vector(imu.angular_vel))
#             R_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])  # Tune as needed.
#             ekf_imu = SkeletonsAVStack.Localization.ekf_update_imu(ekf_imu, z_imu, R_imu)
#         end
        
#         # Fuse the two branches: take position and orientation from the GPS branch (fields 1:7),
#         # and velocity and angular velocity from the IMU branch (fields 8:13).
#         fused = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
        
#         # For debugging, you may print the fused state.
#         # println("Fused state: ", fused.x)
        
#         # Remove (take) any stale state from the channel, then publish the new state.
#         if !isempty(localization_state_channel)
#             take!(localization_state_channel)
#         end
#         put!(localization_state_channel, fused.x)
#         sleep(0.01)  # adjust loop rate as needed.
#     end
# end
# function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)
#     # --- Initialization: Wait for the first GPS measurement ---
#     println("Waiting for initial GPS measurement for EKF initialization...")
#     while !isready(gps_channel)
#         if isready(shutdown_channel) && fetch(shutdown_channel)
#             return  # Exit early if shutdown is signalled.
#         end
#         sleep(0.01)
#     end
#     initial_gps = take!(gps_channel)
#     println("Initial GPS measurement: ", initial_gps)
    
#     # Use a known segment from the map as the reference.
#     # Here, we choose segment 1 (adjust as needed).
#     chosen_seg = 1
#     # Get the initialization point as defined in the map (returns a CarConfig).
#     init_config = VehicleSim.get_initialization_point(VehicleSim.city_map()[chosen_seg])
#     ref_state = init_config.position  # This is an SVector{3,Float64} with x, y, z in meters.
    
#     # Use the first received GPS measurement as the reference for raw GPS.
#     ref_gps = initial_gps
#     println("Reference GPS (raw): ", ref_gps)
#     println("Map initialization position (ref_state): ", ref_state)
    
#     # Set up the state vector (13-dimensional):
#     x0 = zeros(13)
#     # Initialize position using the map's origin.
#     x0[1] = ref_state[1]
#     x0[2] = ref_state[2]
#     x0[3] = ref_state[3]
    
#     println("Initial state position (x,y): ", x0[1:2])
    
#     # Set orientation as identity quaternion (you might also use init_config.yaw if desired).
#     x0[4] = 1.0; x0[5:7] .= 0.0
#     # Initialize velocity and angular velocity to zero.
#     x0[8:13] .= 0.0
#     P0 = 0.1 * Matrix(1.0I, 13, 13)
    
#     ekf_gps = SkeletonsAVStack.Localization.EKFState(x0, P0)
#     ekf_imu = SkeletonsAVStack.Localization.EKFState(x0, P0)
    
#     fused_state = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
#     put!(localization_state_channel, fused_state.x)
#     println("Initial fused state: ", fused_state.x)
    
#     t_last = time()
    
#     # Main loop: update the state continuously.
#     while true
#         if isready(shutdown_channel) && fetch(shutdown_channel)
#             println("Shutdown signal received. Exiting localization loop.")
#             break
#         end

#         # Collect fresh measurements.
#         fresh_gps_meas = []
#         while isready(gps_channel)
#             push!(fresh_gps_meas, take!(gps_channel))
#         end
#         fresh_imu_meas = []
#         while isready(imu_channel)
#             push!(fresh_imu_meas, take!(imu_channel))
#         end
        
#         dt = time() - t_last
#         t_last = time()
        
#         ekf_gps = SkeletonsAVStack.Localization.ekf_predict(ekf_gps, dt)
#         ekf_imu = SkeletonsAVStack.Localization.ekf_predict(ekf_imu, dt)
        
#         # Process all fresh GPS measurements.
#         # Here we convert raw GPS differences (assumed in degrees) to meters.
#         # For latitude: ~111320 meters per degree.
#         # For longitude: ~111320*cos(latitude_in_radians) meters per degree.
#         for gps in fresh_gps_meas
#             delta_lat = gps.lat - ref_gps.lat
#             delta_long = gps.long - ref_gps.long
#             dx = delta_lat 
#             dy = delta_long * cos(ref_gps.lat * π/180.0)
#             local_x = ref_state[1] + dx
#             local_y = ref_state[2] + dy
#             # Use the GPS heading as given.
#             z_gps = [local_x, local_y, gps.heading]
#             R_gps = Diagonal([1.0, 1.0, 0.1])
#             ekf_gps = SkeletonsAVStack.Localization.ekf_update_gps(ekf_gps, z_gps, R_gps)
#         end
        
#         # Process all fresh IMU measurements.
#         for imu in fresh_imu_meas
#             z_imu = vcat(Vector(imu.linear_vel), Vector(imu.angular_vel))
#             R_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
#             ekf_imu = SkeletonsAVStack.Localization.ekf_update_imu(ekf_imu, z_imu, R_imu)
#         end
        
#         fused = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
#         if !isempty(localization_state_channel)
#             take!(localization_state_channel)
#         end
#         put!(localization_state_channel, fused.x)
#         sleep(0.01)  # adjust loop rate as needed.
#     end
# end

function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)
    # --- Initialization: Wait for the first GPS measurement ---
    println("Waiting for initial GPS measurement for EKF initialization...")
    while !isready(gps_channel)
        if isready(shutdown_channel) && fetch(shutdown_channel)
            return  # Exit early if shutdown is signalled.
        end
        sleep(0.01)
    end
    initial_gps = take!(gps_channel)
    println("Initial GPS measurement: ", initial_gps)
    
    # Use a chosen segment (say, segment 1) from the city map to establish the map’s coordinate frame.
    chosen_seg = 1
    init_config = VehicleSim.get_initialization_point(VehicleSim.city_map()[chosen_seg])
    map_origin = init_config.position  # An SVector{3} of (x,y,z) in meters.
    println("Map initialization position from chosen segment: ", map_origin)
    
    # Compute the offset between the map's expected origin and the raw GPS measurement.
    # (Assume raw GPS measurement fields, though named lat/long, are expressed in meters in a different frame.)
    raw_gps_pos = [initial_gps.lat, initial_gps.long, 0.0]
    offset = map_origin - raw_gps_pos
    println("Computed offset between map origin and raw GPS: ", offset)
    
    # Initialize the 13-dimensional state vector.
    x0 = zeros(13)
    # Set the initial position to the map origin.
    x0[1] = map_origin[1]
    x0[2] = map_origin[2]
    x0[3] = map_origin[3]
    
    println("Initialized state position (x,y): ", x0[1:2])
    
    # Initialize orientation as identity quaternion (or use init_config.yaw if available).
    x0[4] = 1.0; x0[5:7] .= 0.0
    # Initialize velocities to zero.
    x0[8:13] .= 0.0
    P0 = 0.1 * Matrix(1.0I, 13, 13)
    
    # Create initial EKF states.
    ekf_gps = SkeletonsAVStack.Localization.EKFState(x0, P0)
    ekf_imu = SkeletonsAVStack.Localization.EKFState(x0, P0)
    
    # Fuse the two branches and publish the initial state.
    fused_state = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
    put!(localization_state_channel, fused_state.x)
    println("Initial fused state: ", fused_state.x)
    
    t_last = time()
    
    # Main loop: update continuously.
    while true
        if isready(shutdown_channel) && fetch(shutdown_channel)
            println("Shutdown signal received. Exiting localization loop.")
            break
        end

        # Collect fresh GPS and IMU measurements.
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
        
        # Prediction step for both EKF branches.
        ekf_gps = SkeletonsAVStack.Localization.ekf_predict(ekf_gps, dt)
        ekf_imu = SkeletonsAVStack.Localization.ekf_predict(ekf_imu, dt)
        
        # Process all fresh GPS measurements.
        # Apply the previously computed offset to translate raw GPS reading into the map frame.
        for gps in fresh_gps_meas
            raw_pos = [gps.lat, gps.long, 0.0]
            corrected_pos = raw_pos + offset   # now in map coordinates.
            z_gps = [corrected_pos[1], corrected_pos[2], gps.heading]
            R_gps = Diagonal([1.0, 1.0, 0.1])
            ekf_gps = SkeletonsAVStack.Localization.ekf_update_gps(ekf_gps, z_gps, R_gps)
        end
        
        # Process all fresh IMU measurements (unchanged).
        for imu in fresh_imu_meas
            z_imu = vcat(Vector(imu.linear_vel), Vector(imu.angular_vel))
            R_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
            ekf_imu = SkeletonsAVStack.Localization.ekf_update_imu(ekf_imu, z_imu, R_imu)
        end
        
        # Fuse the two branches.
        fused = SkeletonsAVStack.Localization.fuse_states(ekf_gps, ekf_imu)
        if !isempty(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, fused.x)
        sleep(0.01)
    end
end





function perception(cam_meas_channel, perception_state_channel, shutdown_channel)
    while true
        if isready(shutdown_channel) && fetch(shutdown_channel)
            break
        end
        
        # Collect all fresh camera measurements.
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            push!(fresh_cam_meas, take!(cam_meas_channel))
        end
        
        # Process each camera measurement.
        all_tracked = Perception.TrackedObject[]
        for cam_meas in fresh_cam_meas
            tracked_objects = Perception.process_camera_measurement(cam_meas)
            append!(all_tracked, tracked_objects)
        end
        # In a full system, you might run a tracking filter for each detected object
        # to compute velocities and associate detections over time. Here, we simply pass the detections.
        perception_state = all_tracked
        
        # Publish the new perception state.
        if !isempty(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
        
        # For debugging, print each tracked object.
        # if !isempty(perception_state)
        #     println("Perception: Published tracked objects:")
        #     for obj in perception_state
        #         println("  Object $(obj.id) in camera frame: Position = $(obj.position), BBox = $(obj.bbox)")
        #     end
        # else
        #     println("Perception: No objects detected.")
        # end
        
        sleep(0.05)  # Adjust loop rate as needed.
    end
end

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

# function decision_making(localization_state_channel, perception_state_channel, target_segment_channel, shutdown_channel, road_map, socket)
#     while true
#         fetch(shutdown_channel) && break

#         # Get the latest localization state (fused state from the EKF).
#         latest_loc = fetch(localization_state_channel)  # 13-element state vector.
#         # Assume that latest_loc[1:2] now holds the vehicle's local x and y (in meters).
#         vehicle_pos = latest_loc[1:2]
#         # Compute vehicle heading from the quaternion in elements 4:7.
#         vehicle_heading = VehicleSim.extract_yaw_from_quaternion(latest_loc[4:7])
#         # Determine the current segment by finding which segment's midpoint is closest.
#         current_seg = find_nearest_segment(vehicle_pos, road_map)

#         # (Optional) Print for debugging:
#         # println("Vehicle position: ", vehicle_pos, " approximated to segment: ", current_seg)

#         # Check perception: get tracked objects and decide if any obstacle is dangerously close.
#         perception_state = []
#         if isready(perception_state_channel)
#             perception_state = take!(perception_state_channel)
#         end
#         obstacle_detected = false
#         for obj in perception_state
#             if (obj.position[3] < 10.0) && (abs(obj.position[1]) < 2.0)
#                 obstacle_detected = true
#                 break
#             end
#         end

#         # Fetch the target segment from target_segment_channel.
#         target_seg = fetch(target_segment_channel)
        
#         # Compute the route using your Routing module.
#         route = SkeletonsAVStack.Routing.compute_route(road_map, current_seg, target_seg)
        
#         # Determine if the upcoming road requires a forced stop.
#         should_stop_stop_sign = false
#         if length(route) >= 2
#             next_seg = road_map[route[2]]
#             if SkeletonsAVStack.Routing.segment_type(next_seg) == VehicleSim.stop_sign
#                 seg_pos = (next_seg.lane_boundaries[1].pt_a + next_seg.lane_boundaries[1].pt_b)/2
#                 dist_to_stop = norm(vehicle_pos - seg_pos)
#                 if dist_to_stop < 5.0  # 5米范围内才触发停止
#                     should_stop_stop_sign = true
#                 end
#             end
#         end

#         # Determine whether we should park in the loading zone.
#         should_park = false
#         if haskey(road_map, target_seg)
#             seg = road_map[target_seg]
#             if VehicleSim.reached_target(vehicle_pos, 0.0, seg)
#                 should_park = true
#             end
#         end

#         # Generate nominal control commands from motion planning.
#         if isempty(route)
#             steering_angle = 0.0
#             target_vel = 0.0
#         else
#             # call motion planning to obtain a steering command
#             lookahead_point, steering_angle = SkeletonsAVStack.MotionPlanning.plan_and_control(road_map, route, vehicle_pos, vehicle_heading; lookahead_distance=0.5)
#             # target_vel = road_map[target_seg].speed_limit
#             target_vel = 2
#             println("Planned steering angle: ", steering_angle, " | Target vel: ", target_vel, "| lookahead_point", lookahead_point)
#         end

#         # Override commands if an obstacle is detected in perception.
#         if obstacle_detected
#             println("Decision-making: Obstacle detected! Stopping vehicle.")
#             target_vel = 0.0
#         end

#         # Override commands if the next segment is a stop sign.
#         if should_stop_stop_sign
#             println("Decision-making: Approaching stop sign. Stopping and waiting 3 seconds...")
#             target_vel = 0.0
#         end

#         # Override commands if parking is required.
#         if should_park
#             println("Decision-making: In loading zone. Parking...")
#             target_vel = 0.0
#         end

#         # Package and send the command.
#         cmd = (steering_angle, target_vel, true)
#         if should_stop_stop_sign
#             stop_start_time = time()
#             while time() - stop_start_time < 3.0
#                 # 仍可以处理其他消息
#                 sleep(0.1)
#             end
#         end
#         # println("Sending command: ", cmd)  # 添加调试输出
#         serialize(socket, cmd)
#         # println("Vehicle pos: ", vehicle_pos, " | Current seg: ", current_seg, " | Route: ", route)
#     end
# end
# Helper function to check if a road segment is curved.
function is_curved(seg::VehicleSim.RoadSegment; atol=1e-6)
    for lb in seg.lane_boundaries
        if !isapprox(lb.curvature, 0.0; atol=atol)
            return true
        end
    end
    return false
end


# Revised decision-making function that computes the route once.
function decision_making(localization_state_channel, perception_state_channel, 
                         target_segment_channel, shutdown_channel, 
                         road_map::Dict{Int,VehicleSim.RoadSegment}, socket)
    # --- Initial Routing (compute once) ---
    # Wait a short time to get an initial state measurement.
    while !isready(localization_state_channel)
        sleep(0.01)
    end
    initial_state = take!(localization_state_channel)
    vehicle_pos = initial_state[1:2]
    vehicle_heading = VehicleSim.extract_yaw_from_quaternion(initial_state[4:7])
    
    # Get the target segment from its channel.
    target_seg_id = fetch(target_segment_channel)
    # Find the starting segment (for example, using your find_nearest_segment helper).
    start_seg_id = find_nearest_segment(vehicle_pos, road_map)
    
    # Compute the route once from the starting segment to the target.
    route = SkeletonsAVStack.Routing.compute_route(road_map, start_seg_id, target_seg_id)
    if isempty(route)
        println("No valid route found from segment $start_seg_id to $target_seg_id. Aborting route execution.")
        return
    end
    println("Computed route: ", route)
    # Set index to the current active segment in the route.
    current_index = 1

    # --- Main Loop: Follow the Precomputed Route ---
    while true
        if isready(shutdown_channel) && fetch(shutdown_channel)
            break
        end

        # Get the latest localization state.
        while !isready(localization_state_channel)
            sleep(0.001)
        end
        latest_loc = take!(localization_state_channel)
        vehicle_pos = latest_loc[1:2]
        vehicle_heading = VehicleSim.extract_yaw_from_quaternion(latest_loc[4:7])
        # Determine the current segment from the precomputed route.
        current_seg_id = route[current_index]
        current_seg = road_map[current_seg_id]
        # Check if the vehicle has reached the target of this segment.
        # (Assumes your reached_target(pos, vel, seg) function returns true if the vehicle is
        #  within a pre-defined threshold of the segment’s end.)
        seg_id = find_nearest_segment(vehicle_pos, road_map)
        if  seg_id !== nothing
            println("Segment $seg_id reached.")
            # If there is a next segment, advance the route index.
            if  seg_id == route[current_index+1]
                current_index += 1
                current_seg_id = route[current_index]
                current_seg = road_map[current_seg_id]
            elseif current_index == length(route)
                # Reached the end of the route (target segment).
                println("Target segment reached. Stopping vehicle.")
                serialize(socket, (0.0, 0.0, true))
                break
            end
        end
        # Determine the desired steering and target velocity.
        steering_angle = 0.0
        target_vel = 7.0  # A default speed—you may choose to vary this.
        # If the current segment is straight, check if the next segment (if available) is curved.
        if !is_curved(current_seg)
            if current_index < length(route)
                next_seg = road_map[route[current_index + 1]]
                if is_curved(next_seg)
                    # Compute lookahead point as, for example, the midpoint of the pt_b endpoints
                    # of the first two lane boundaries of the curved segment.
                    lb1 = next_seg.lane_boundaries[1]
                    lb2 = next_seg.lane_boundaries[2]
                    lookahead_point = 0.5 * (lb1.pt_b .+ lb2.pt_b)
                    desired_heading = atan(lookahead_point[2] - vehicle_pos[2],
                                           lookahead_point[1] - vehicle_pos[1])
                    steering_angle = desired_heading - vehicle_heading
                else
                    steering_angle = 0.0
                end
            else
                # No next segment, so drive straight.
                steering_angle = 0.0
            end
        else
            # For curved segments, you may use a more sophisticated controller.
            # For example, use your motion planner as a fallback:
            (lookahead_point, steering_angle) = SkeletonsAVStack.MotionPlanning.plan_and_control(
                                                    road_map, route, vehicle_pos, vehicle_heading;
                                                    lookahead_distance=0.5)
        end
        
        # (Optional) Add any perception-based overrides here (for obstacles, stop signs, etc.)
        # For example, see your previous code that might set target_vel = 0.0 if an obstacle is detected.
        
        println("Vehicle pos: ", vehicle_pos, " | Current seg: ", current_seg_id,
                " | Route index: ", current_index, " | Steering angle: ", steering_angle,
                " | Target vel: ", target_vel)
        
        # Package and send the command.
        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
        sleep(0.01)  # adjust loop rate as needed
    end
end



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
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)

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
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    if use_gt
        @async process_gt(gt_channel,
                      shutdown_channel, target_segment_channel, 
                      perception_state_channel, map_segments, socket)
    else
        @async localize(gps_channel, 
                    imu_channel, 
                    localization_state_channel, 
                    shutdown_channel)

        @async perception(cam_channel,
                      perception_state_channel, 
                      shutdown_channel)
    end



    @async decision_making(localization_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map_segments, 
                           socket)
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