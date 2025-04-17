#!/usr/bin/env julia
# test_motion_planning.jl

include("../src/SkeletonsAVStack.jl")
using .SkeletonsAVStack
using .SkeletonsAVStack.MotionPlanning
using VehicleSim    # For accessing map generation functions and RoadSegment definitions.

function test_motion_planning()
    # For testing, use a more connected map (city_map) so that the route is nontrivial.
    map = VehicleSim.city_map()
    # For example, compute the route from the first to the last segment:
    seg_ids = collect(keys(map))
    start_seg = seg_ids[1]
    goal_seg = seg_ids[end]
    route = SkeletonsAVStack.Routing.compute_route(map, start_seg, goal_seg)
    
    println("Route computed: ", route)
    
    # Let’s assume the vehicle is at a certain starting position and heading.
    vehicle_pos = [0.0, 0.0]  # Example starting coordinate.
    vehicle_heading = 0.0     # Facing east, for instance.
    
    # Compute the lookahead point and steering command.
    lookahead_point, steering_cmd = plan_and_control(map, route, vehicle_pos, vehicle_heading; lookahead_distance=5.0)
    
    println("Lookahead point: ", lookahead_point)
    println("Steering command: ", steering_cmd)
end

test_motion_planning()
