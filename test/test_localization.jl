# #!/usr/bin/env julia
# # test/test_localization.jl

# # Include the main project module so that all submodules (including Localization)
# # are loaded. The relative path here assumes that SkeletonsAVStack.jl is in src/.
# include("../src/SkeletonsAVStack.jl")
# using .SkeletonsAVStack       # Bring the main module into scope.
# using VehicleSim              # For GPSMeasurement and IMUMeasurement.
# using StaticArrays            # For SVector.

# # Create channels for sensor measurements and localization state.
# const gps_channel = Channel{VehicleSim.GPSMeasurement}(10)
# const imu_channel = Channel{VehicleSim.IMUMeasurement}(10)
# const localization_state_channel = Channel{Vector{Float64}}(10)
# const shutdown_channel = Channel{Bool}(1)

# # -------------------------------------
# # Simulate sensor measurements.
# # -------------------------------------
# function simulate_measurements()
#     # Push an initial GPS measurement (time, lat, long, heading).
#     push!(gps_channel, VehicleSim.GPSMeasurement(time(), 0.0, 0.0, 0.0))
#     # Then, every 0.1 seconds, push an IMU measurement.
#     while true
#         sleep(0.1)
#         # For example, assume constant forward motion (1.0 m/s along the x direction) and no rotation.
#         push!(imu_channel, VehicleSim.IMUMeasurement(time(), SVector(1.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0)))
#     end
# end

# # Start sensor simulation in an asynchronous task.
# @async simulate_measurements()

# # -------------------------------------
# # Launch localization routine.
# # -------------------------------------
# # We run the localize() function (which implements the dual–EKF and fusion process)
# # in an asynchronous task. It will read from the gps_channel and imu_channel,
# # update its internal estimates, and push the fused state to localization_state_channel.
# @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

# # -------------------------------------
# # Shutdown control: After a fixed time, signal shutdown.
# # -------------------------------------
# @async begin
#     # Let the localization run for, say, 3 seconds.
#     sleep(3.0)
#     put!(shutdown_channel, true)
#     println("Shutdown signal issued.")
# end

# # -------------------------------------
# # Monitor and print the localization state.
# # -------------------------------------
# # Loop until the shutdown channel eventually signals shutdown.
# start_time = time()
# while isopen(localization_state_channel)
#     if isready(localization_state_channel)
#         fused_state = take!(localization_state_channel)
#         println("Fused localization state: ", fused_state)
#     end
#     sleep(0.2)
#     # Optionally, break the loop if enough time has passed
#     if time() - start_time > 4.0
#         break
#     end
# end

# println("Localization test complete!")

#!/usr/bin/env julia
# test/test_localization.jl

# Include the main project module.
include("../src/SkeletonsAVStack.jl")
using .SkeletonsAVStack         # Bring the main module into scope.
using VehicleSim                # For GPSMeasurement and IMUMeasurement types.
using StaticArrays              # For SVector

# Create channels for sensor data and for publishing the fused state.
const gps_channel = Channel{VehicleSim.GPSMeasurement}(10)
const imu_channel = Channel{VehicleSim.IMUMeasurement}(10)
const localization_state_channel = Channel{Vector{Float64}}(10)
const shutdown_channel = Channel{Bool}(1)

# --------------------------------------------------
# Simulate sensor measurements: push initial and continuous values.
# --------------------------------------------------
function simulate_measurements()
    println("Simulate: sending initial GPS measurement")
    # Send an initial GPS measurement.
    put!(gps_channel, VehicleSim.GPSMeasurement(time(), 0.0, 0.0, 0.0))
    while true
        sleep(0.1)
        # Always push an IMU measurement.
        put!(imu_channel, VehicleSim.IMUMeasurement(time(), SVector(1.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0)))
        # Occasionally push a GPS measurement.
        if rand() < 0.2
            put!(gps_channel, VehicleSim.GPSMeasurement(time(), 0.0 + 0.1 * randn(), 0.0 + 0.1 * randn(), 0.0))
        end
    end
end

@async simulate_measurements()

# --------------------------------------------------
# Launch the localization routine as an asynchronous task.
# --------------------------------------------------
# Note: Make sure your localize() function checks shutdown by testing for emptiness of shutdown_channel
# (e.g. using isempty(shutdown_channel)) rather than consuming a value with fetch().
localize_task = @async SkeletonsAVStack.Localization.localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel)

# Let localization run for 5 seconds.
sleep(5.0)
put!(shutdown_channel, true)
println("Shutdown signal issued.")

# Give the localize() task a moment to process the shutdown and exit.
sleep(0.5)

# Drain and print any fused localization state published.
println("Fused localization state(s):")
while isready(localization_state_channel)
    fused_state = take!(localization_state_channel)
    println(fused_state)
end

println("Localization test complete!")
