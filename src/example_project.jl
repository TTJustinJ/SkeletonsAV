struct MyLocalizationType
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

#Original dummy function
"""
function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end
"""

function localize(gps_channel, imu_channel, localization_state_channel)
    # Wait for the first GPS measurement to initialize the state.
    gps_meas = take!(gps_channel)
    # Initialize state vector x ∈ ℝ¹³:
    # x[1:3]   = position (we use lat, long, and set z = 0)
    x = zeros(13)
    x[1] = gps_meas.lat
    x[2] = gps_meas.long
    x[3] = 0.0

    # For a planar vehicle, assume quaternion from yaw = heading:
    # x[4:7]   = quaternion [w, x, y, z] from the GPS heading (assume pitch and roll are zero)
    yaw = gps_meas.heading
    x[4] = cos(yaw/2)   # w
    x[5] = 0.0          # x
    x[6] = 0.0          # y
    x[7] = sin(yaw/2)   # z

    # Initialize velocity and angular velocity to zero.
    # x[8:10]  = velocity
    # x[11:13] = angular velocity
    x[8:10] .= 0.0
    x[11:13] .= 0.0

    # Initial covariance matrix for the state
    P = Diagonal(fill(0.1, 13))

    last_time = gps_meas.time

    while true
        # Get the current system time (or use the next available measurement timestamp)
        current_time = time()
        dt = current_time - last_time
        if dt <= 0
            dt = 0.01  # ensure a small time step
        end

        # Process all available IMU measurements
        while isready(imu_channel)
            imu_meas = take!(imu_channel)
            dt_imu = imu_meas.time - last_time
            dt = max(dt, dt_imu)
            # EKF Prediction Step:
            x = f(x, dt)
            F = Jac_x_f(x, dt)
            # Define process noise covariance Q (tune as necessary)
            Q = Diagonal(fill(0.01, 13))
            P = F * P * F' + Q
            last_time = imu_meas.time
        end

        # Process a GPS measurement
        if isready(gps_channel)
            gps_meas = take!(gps_channel)
            # Measurement vector: [lat, long, heading]
            z = [gps_meas.lat, gps_meas.long, gps_meas.heading]
            # Predicted measurement from current state:
            z_pred = h_gps(x)
            # Innovation:
            y = z - z_pred
            # Measurement Jacobian:
            H = Jac_h_gps(x)
            # Measurement noise covariance (tune these values)
            R = Diagonal([1.0, 1.0, 0.1])
            S = H * P * H' + R
            K = P * H' * inv(S)
            # EKF Correction Step:
            x = x + K * y
            P = (I(13) - K * H) * P
            last_time = gps_meas.time
        end

        # Publish the localization state.
        loc_state = MyLocalizationType(round(Int, x[1]), x[2])
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, loc_state)

        sleep(0.001)
    end
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 0.0
        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    #localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

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

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end
