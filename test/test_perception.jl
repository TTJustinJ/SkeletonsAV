#!/usr/bin/env julia
# test/test_perception.jl

# Include the main project module to load all submodules (Perception, etc.)
include("../src/SkeletonsAVStack.jl")
using .SkeletonsAVStack           # Bring your project module into scope.
using VehicleSim                  # For the CameraMeasurement type.
using StaticArrays                # For SVector types.

# Create channels:
# - cam_meas_channel: for simulated CameraMeasurement objects
# - localization_state_channel: for a (dummy) localization state (needed by the perception function)
# - perception_state_channel: where the perception routine will publish tracked objects
# - shutdown_channel: used to signal termination.
const cam_meas_channel = Channel{VehicleSim.CameraMeasurement}(10)
const localization_state_channel = Channel{Vector{Float64}}(1)
const perception_state_channel = Channel{Any}(10)    # You could use a more specific type if desired.
const shutdown_channel = Channel{Bool}(1)

# Define a helper function to simulate a camera measurement.
function simulate_cam_measurement()
    # Create a dummy bounding box:
    #   [top, left, bottom, right].
    # For instance, a detection roughly in the center:
    bbox = @SVector [200, 300, 400, 340]   # height ~200 pixels, width ~40 pixels.
    return VehicleSim.CameraMeasurement(time(), 1, 0.64, 0.001, 640, 480, [bbox])
end

# Launch the perception routine as an asynchronous task.
perception_task = @async SkeletonsAVStack.Perception.perception(cam_meas_channel, perception_state_channel, shutdown_channel)

# Simulate sending a few camera measurements.
@async begin
    for i in 1:5
        sleep(0.1)
        put!(cam_meas_channel, simulate_cam_measurement())
    end
    sleep(0.5)
    put!(shutdown_channel, true)
    println("Shutdown signal issued.")
end

# Monitor and print the outputs from the perception_state_channel for a short duration.
start_time = time()
while time() - start_time < 2.0
    if isready(perception_state_channel)
        tracked_objs = take!(perception_state_channel)
        println("Test Perception: Published tracked objects:")
        for obj in tracked_objs
            println("  Object $(obj.id): Position = $(obj.position), BBox = $(obj.bbox)")
        end
    end
    sleep(0.1)
end

println("Perception test complete!")
