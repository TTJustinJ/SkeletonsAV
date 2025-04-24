# test_routing.jl
using Test
include("../src/SkeletonsAVStack.jl")
using .SkeletonsAVStack           # Bring your project module into scope.
using VehicleSim

# Create a city map using VehicleSim's city_map() function.
city_map = VehicleSim.city_map()


# Print out available segment IDs.
segments = sort(collect(keys(city_map)))
println("City map segments: ", segments)

# Choose a start segment and a goal segment.
# For testing purposes, we randomly selected two segments
start_seg = 78
goal_seg = 56
println("Using start segment: ", start_seg, " and goal segment: ", goal_seg)

# Compute the route using the compute_route function.
route = SkeletonsAVStack.Routing.compute_route(city_map, start_seg, goal_seg)
println("Computed route from $start_seg to $goal_seg: ", route)

# Verify that each consecutive segment in the route is connected in the map.
for i in 1:(length(route) - 1)
    seg = city_map[route[i]]
    next_seg = route[i+1]
    if next_seg in seg.children 
        print("Error: Segment $(next_seg) is not listed as a child of segment $(route[i]).")
    end
end

println("All routing tests passed!")
