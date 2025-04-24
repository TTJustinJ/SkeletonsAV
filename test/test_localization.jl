
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
