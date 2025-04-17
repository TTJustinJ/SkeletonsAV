
# ---------------------------------------------------------------------------
# State Types
# ---------------------------------------------------------------------------
# We now define an EKF state for localization.
# The state vector is 13-dimensional:
#   x[1:3]   : position (in map frame)
#   x[4:7]   : orientation as quaternion [w, x, y, z]
#   x[8:10]  : velocity (in body frame)
#   x[11:13] : angular velocity (in body frame)
struct EKFState
    x::Vector{Float64}    # 13-element state vector
    P::Matrix{Float64}    # 13x13 covariance matrix
end

# Dummy perception state remains simple.
struct MyPerceptionType
    field1::Int
    field2::Float64
end

# ---------------------------------------------------------------------------
# EKF and Process/Measurement Models
# ---------------------------------------------------------------------------
# We use the functions defined in measurements.jl:
#   - f(x, Δt) and Jac_x_f(x, Δt): process model and its Jacobian.
#   - h_gps(x) and Jac_h_gps(x): measurement model (for GPS) and its Jacobian.
#
# We define Q and R for our EKF:
const Q_ekf = Diagonal(fill(0.01, 13))  # Process noise covariance (tune as needed)
const R_ekf = Diagonal([0.5, 0.5, 0.1])   # GPS measurement noise covariance

# Updated EKF update function.
# It first propagates the state using available IMU measurements (by calling f)
# and then, if a GPS measurement is available, applies an update step.
function ekf_update(prev_ekf::EKFState, gps_meas::Vector{GPSMeasurement}, imu_meas::Vector{IMUMeasurement}, dt::Float64)
    ekf = prev_ekf
    # Process each IMU measurement by propagating the state.
    # (In a full implementation, you’d incorporate the IMU values into the process model;
    # here we simply call the provided rigid-body dynamics function f.)
    for meas in imu_meas
        # For simplicity, we use a fixed dt for each IMU update.
        x_pred = f(ekf.x, dt)
        A = Jac_x_f(ekf.x, dt)
        P_pred = A * ekf.P * A' + Q_ekf
        ekf = EKFState(x_pred, P_pred)
    end

    # If a GPS measurement is available, perform the update.
    if !isempty(gps_meas)
        # Use the most recent GPS measurement.
        latest_gps = gps_meas[end]
        # Convert the GPS measurement (lat, long, heading) into a measurement vector.
        # Here we assume that lat and long are already expressed in map-frame coordinates.
        z = [latest_gps.lat, latest_gps.long, latest_gps.heading]
        z_pred = h_gps(ekf.x)
        y = z - z_pred
        H = Jac_h_gps(ekf.x)
        S = H * ekf.P * H' + R_ekf
        K = ekf.P * H' * inv(S)
        x_new = ekf.x + K * y
        P_new = (I - K * H) * ekf.P
        ekf = EKFState(x_new, P_new)
    end
    return ekf
end

# ---------------------------------------------------------------------------
# Perception Update Function (Dummy)
# ---------------------------------------------------------------------------
# We update the perception state based on CameraMeasurement data.
# In this dummy implementation we count the total number of bounding boxes
# and combine that with the norm of the current estimated position.
function perception_update(cam_meas, loc_state::EKFState)
    count = 0
    for meas in cam_meas
        count += length(meas.bounding_boxes)
    end
    # Use the norm of the estimated position as an additional value.
    pos_norm = norm(loc_state.x[1:3])
    return MyPerceptionType(count, pos_norm)
end

# ---------------------------------------------------------------------------
# Processing Functions
# ---------------------------------------------------------------------------
# Process ground-truth measurements (for debugging/cheating mode).
function process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel)
    while true
        fetch(shutdown_channel) && break

        fresh_gt_meas = Measurement[]
        while isready(gt_channel)
            push!(fresh_gt_meas, take!(gt_channel))
        end

        # In GT mode we “cheat” by setting a fixed state.
        # For example, we set the EKF state to a predetermined value.
        new_state = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        new_cov = Matrix{Float64}(I, 13, 13)
        new_localization_state = EKFState(new_state, new_cov)
        new_perception_state = MyPerceptionType(1, 1.0)

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, new_localization_state)

        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, new_perception_state)
    end
end

# Localization function using the updated EKF.
function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)
    # Initialize EKF state:
    # Start at position (0,0,0), orientation as quaternion [1, 0, 0, 0],
    # and zero velocity and angular velocity.
    x0 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    P0 = Matrix{Float64}(I, 13, 13)
    ekf_state = EKFState(x0, P0)
    dt = 0.1  # time step (seconds)

    while true
        fetch(shutdown_channel) && break

        fresh_gps_meas = GPSMeasurement[]
        while isready(gps_channel)
            push!(fresh_gps_meas, take!(gps_channel))
        end

        fresh_imu_meas = IMUMeasurement[]
        while isready(imu_channel)
            push!(fresh_imu_meas, take!(imu_channel))
        end

        ekf_state = ekf_update(ekf_state, fresh_gps_meas, fresh_imu_meas, dt)

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, ekf_state)
    end 
end

# Perception processing: project camera measurements and update perception state.
function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    while true
        fetch(shutdown_channel) && break

        fresh_cam_meas = CameraMeasurement[]
        while isready(cam_meas_channel)
            push!(fresh_cam_meas, take!(cam_meas_channel))
        end

        latest_localization_state = fetch(localization_state_channel)
        perception_state = perception_update(fresh_cam_meas, latest_localization_state)

        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

# Decision making: integrate localization and perception to compute control commands.
# Here we compute a dummy steering angle based on the current position.
function decision_making(localization_state_channel, perception_state_channel, target_segment_channel,
                         shutdown_channel, map, socket)
    while true
        fetch(shutdown_channel) && break

        latest_localization_state = fetch(localization_state_channel)  # EKFState
        latest_perception_state = fetch(perception_state_channel)      # MyPerceptionType

        # For demonstration, compute steering angle as the angle from current position
        # toward a fixed target point from the map. (In a full system, route planning via Dijkstra would be used.)
        # Here, we extract a dummy target point from the map (assume map is a Dict with keys "segment1", etc.)
        # and use the localization state's position.
        current_position = latest_localization_state.x[1:2]
        # Assume the map contains a target point; if not, default to (0,0).
        target_point = haskey(map, "target") ? map["target"] : SVector(10.0, 10.0)
        dx = target_point[1] - current_position[1]
        dy = target_point[2] - current_position[2]
        steering_angle = atan(dy, dx)
        # Dummy target velocity from the perception state.
        target_vel = latest_perception_state.field2

        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
        sleep(0.1)
    end
end

# Check if a channel is full.
function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end

# Dummy nonblocking key input.
function get_c()
    return ' '  # Replace with real nonblocking key input if needed.
end

# ---------------------------------------------------------------------------
# Client Setup and Shutdown
# ---------------------------------------------------------------------------
module VehicleSim
export city_map
function city_map()
    # Use the city_map function from map.jl.
    return city_map()
end
end

function my_client(host::IPAddr=IPv4(0), port::Int=4444; use_gt=false)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()  # Road network from map.jl
    
    msg = deserialize(socket)  # Visualization info
    @info msg

    # Create channels using the updated measurement types.
    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    # Use EKFState for localization.
    localization_state_channel = Channel{EKFState}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    target_map_segment = 0  # (will be overwritten by incoming messages)
    ego_vehicle_id = 0      # (will be overwritten by incoming messages)
    put!(target_segment_channel, target_map_segment)

    @async begin
        # This task continuously reads from the socket and distributes measurements.
        while true
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
        end
    end

    if use_gt
        @async process_gt(gt_channel, shutdown_channel, localization_state_channel, perception_state_channel)
    else
        @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)
        @async perception(cam_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    end

    @async decision_making(localization_state_channel, perception_state_channel, target_segment_channel,
                           shutdown_channel, map_segments, socket)
end

function shutdown_listener(shutdown_channel)
    info_string = """
    ***************
    CLIENT COMMANDS
    ***************
          - Make sure focus is on this terminal window.
          - Press 'q' to shutdown threads.
    """
    @info info_string
    while true
        sleep(0.1)
        key = get_c()
        if key == 'q'
            take!(shutdown_channel)
            put!(shutdown_channel, true)
            break
        end
    end
end
