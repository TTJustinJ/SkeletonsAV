module Routing

using VehicleSim
using LinearAlgebra

export segment_length, build_graph, dijkstra, compute_route, segment_type

"""
    segment_length(seg::VehicleSim.RoadSegment) -> Float64

Computes the length of a road segment. For straight segments (curvature ≈ 0), it uses the Euclidean
distance between the two endpoints (from the first lane boundary). For curved segments, we assume a 90° turn,
so the length is (π/2) * r, where r = 1/|curvature|.
"""
function segment_length(seg::VehicleSim.RoadSegment)
    lb = seg.lane_boundaries[1]
    if isapprox(lb.curvature, 0.0; atol=1e-6)
        return norm(lb.pt_b - lb.pt_a)
    else
        r = 1.0 / abs(lb.curvature)
        return (π/2) * r
    end
end

"""
    build_graph(road_map::Dict{Int, VehicleSim.RoadSegment}) -> Dict{Int, Vector{Tuple{Int, Float64}}}

Converts the road map (a dictionary of segments) into a graph. Every segment becomes a key with a vector
of outgoing edges. Each edge is a tuple (child_segment_id, cost), where the cost is computed via segment_length.
"""
function build_graph(road_map::Dict{Int, VehicleSim.RoadSegment})
    # Initialize every segment as a key with an empty neighbors list.
    graph = Dict{Int, Vector{Tuple{Int, Float64}}}()
    for (id, _) in road_map
        graph[id] = Tuple{Int, Float64}[]
    end
    # Add directed edges based on the children field.
    for (id, seg) in road_map
        for child_id in seg.children
            if haskey(road_map, child_id)
                cost = segment_length(seg)
                push!(graph[id], (child_id, cost))
            end
        end
    end
    return graph
end

"""
    dijkstra(graph::Dict{Int, Vector{Tuple{Int, Float64}}}, start::Int, goal::Int) -> Vector{Int}

Runs Dijkstra’s algorithm over the graph to find the shortest route from start to goal.
If no route is found, returns an empty vector.
"""
function dijkstra(graph::Dict{Int, Vector{Tuple{Int, Float64}}}, start::Int, goal::Int)
    # Initialize distances and predecessor dictionary
    dist = Dict{Int, Float64}()
    prev = Dict{Int, Int}()
    for node in keys(graph)
        dist[node] = Inf
    end
    dist[start] = 0.0

    unvisited = Set(keys(graph))
    
    while !isempty(unvisited)
        # Find node with smallest distance
        current = first(minimum([(dist[n], n) for n in unvisited])[2])
        
        if current == goal
            break
        end
        
        delete!(unvisited, current)
        
        for (neighbor, cost) in graph[current]
            alt = dist[current] + cost
            if alt < dist[neighbor]
                dist[neighbor] = alt
                prev[neighbor] = current
            end
        end
    end

    # Check if goal is reachable
    if !haskey(prev, goal) && goal != start
        return Int[]
    end

    # Reconstruct path
    path = Int[]
    current = goal
    while current != start
        push!(path, current)
        current = prev[current]
    end
    push!(path, start)
    reverse!(path)
    return path
end



"""
    compute_route(road_map::Dict{Int, VehicleSim.RoadSegment}, start::Int, goal::Int) -> Vector{Int}

Builds the graph from the road map and applies Dijkstra’s algorithm to compute an optimal route from start to goal.
"""
function compute_route(road_map::Dict{Int, VehicleSim.RoadSegment}, start::Int, goal::Int)
    graph = build_graph(road_map)
    return dijkstra(graph, start, goal)
end

"""
    segment_type(seg::VehicleSim.RoadSegment) -> LaneTypes

Returns the primary lane type of the segment (i.e. the first element of the lane_types vector).
"""
function segment_type(seg::VehicleSim.RoadSegment)
    return seg.lane_types[1]
end

end  # module Routing
