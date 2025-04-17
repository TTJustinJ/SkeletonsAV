# module MotionPlanning

# using VehicleSim
# using Interpolations
# using LinearAlgebra

# export plan_trajectory, pure_pursuit_control, plan_and_control

# # --- Preliminary Path Extraction ---
# #
# # Given a RoadSegment, extract a set of points representing the midline.
# # We assume that the segment has at least one lane boundary. If there are two,
# # we compute the midpoint between the corresponding endpoints.

# function extract_midline(seg::VehicleSim.RoadSegment; N::Int=10)
#     # Check if the segment has two boundaries (left and right)
#     if length(seg.lane_boundaries) >= 2
#         lb_left = seg.lane_boundaries[1]
#         lb_right = seg.lane_boundaries[2]
#         # Generate N evenly spaced points between the endpoints of each boundary.
#         ts = LinRange(0.0, 1.0, N)
#         # For each parameter t, compute the midpoint:
#         return [((1-t)*lb_left.pt_a + t*lb_left.pt_b + (1-t)*lb_right.pt_a + t*lb_right.pt_b) / 2 for t in ts]
#     else
#         # If only one boundary exists, generate points along that boundary.
#         lb = seg.lane_boundaries[1]
#         ts = LinRange(0.0, 1.0, N)
#         return [(1-t)*lb.pt_a + t*lb.pt_b for t in ts]
#     end
# end

# # --- Trajectory Generation via Interpolation ---
# #
# # Given a set of segments along a route, extract the waypoint midlines
# # and then use spline interpolation to obtain continuous functions for x and y.

# function plan_trajectory(map::Dict{Int, VehicleSim.RoadSegment}, route::Vector{Int})
#     all_waypoints = []
#     for seg_id in route
#         seg = map[seg_id]
#         wpts = extract_midline(seg, N=10)   # adjust N as needed per segment
#         append!(all_waypoints, wpts)
#     end
#     # Convert the waypoints into separate x and y vectors.
#     xs = [p[1] for p in all_waypoints]
#     ys = [p[2] for p in all_waypoints]
#     n = length(all_waypoints)
#     # Use a simple parameter t; you could alternatively use cumulative arc-length.
#     t = range(1, n, length=n)
    
#     # Create continuous cubic spline interpolations.
#     itp_x = CubicSplineInterpolation(t, xs)
#     itp_y = CubicSplineInterpolation(t, ys)
    
#     # Return the interpolation objects and the parameter range.
#     return (itp_x, itp_y, t)
# end

# # --- Pure Pursuit Control ---
# #
# # Given the vehicle’s current position and heading, along with the trajectory (represented by the interpolations),
# # we locate a lookahead point that is at least a given distance away. Then we compute a steering command.

# function pure_pursuit_control(vehicle_pos::Vector{Float64}, vehicle_heading::Float64,
#                               itp_x, itp_y, t, lookahead_distance::Float64)
#     n = length(t)
#     # Find the first point along the trajectory with distance ≥ lookahead_distance.
#     for i in 1:n
#         p = [itp_x(t[i]), itp_y(t[i])]
#         if norm(p - vehicle_pos) >= lookahead_distance
#             # Compute the relative angle between the vehicle's heading and the vector from the position to p.
#             vec = p - vehicle_pos
#             α = atan(vec[2], vec[1]) - vehicle_heading
#             # Wrap α between -π and π
#             α = atan(sin(α), cos(α))
#             # Example pure pursuit steering angle calculation:
#             L = 2.5  # example wheelbase; adjust based on your vehicle model.
#             steering_angle = atan(2 * L * sin(α) / lookahead_distance)
#             return p, steering_angle
#         end
#     end
#     # If no lookahead point is found, return the last point.
#     p = [itp_x(t[end]), itp_y(t[end])]
#     return p, 0.0
# end

# # --- Combined Planning and Control Function ---
# #
# # Given a map, a pre-computed route, and the current vehicle state,
# # this function generates the trajectory and computes a control command.
# function plan_and_control(map::Dict{Int, VehicleSim.RoadSegment}, route::Vector{Int},
#                           vehicle_pos::Vector{Float64}, vehicle_heading::Float64;
#                           lookahead_distance::Float64 = 5.0)
#     itp_x, itp_y, t = plan_trajectory(map, route)
#     lookahead_point, steering_command = pure_pursuit_control(vehicle_pos, vehicle_heading, itp_x, itp_y, t, lookahead_distance)
#     return (lookahead_point, steering_command)
# end

# end  # module MotionPlanning

module MotionPlanning

using VehicleSim
using Interpolations
using LinearAlgebra

export plan_trajectory, pure_pursuit_control, plan_and_control

function extract_midline(seg::VehicleSim.RoadSegment; N::Int=10)
    if length(seg.lane_boundaries) >= 2
        lb_left = seg.lane_boundaries[1]
        lb_right = seg.lane_boundaries[2]
        ts = LinRange(0.0, 1.0, N)
        return [((1-t)*lb_left.pt_a + t*lb_left.pt_b + (1-t)*lb_right.pt_a + t*lb_right.pt_b)/2 for t in ts]
    else
        lb = seg.lane_boundaries[1]
        ts = LinRange(0.0, 1.0, N)
        return [(1-t)*lb.pt_a + t*lb.pt_b for t in ts]
    end
end

function plan_trajectory(map::Dict{Int, VehicleSim.RoadSegment}, route::Vector{Int})
    all_waypoints = []
    for seg_id in route
        seg = map[seg_id]
        wpts = extract_midline(seg, N=10)
        append!(all_waypoints, wpts)
    end
    xs = [p[1] for p in all_waypoints]
    ys = [p[2] for p in all_waypoints]
    n = length(all_waypoints)
    t = range(1, n, length=n)
    itp_x = CubicSplineInterpolation(t, xs)
    itp_y = CubicSplineInterpolation(t, ys)
    return (itp_x, itp_y, t)
end

function pure_pursuit_control(vehicle_pos::Vector{Float64}, vehicle_heading::Float64, itp_x, itp_y, t, lookahead_distance::Float64)
    n = length(t)
    for i in 1:n
        p = [itp_x(t[i]), itp_y(t[i])]
        if norm(p - vehicle_pos) >= lookahead_distance
            vec = p - vehicle_pos
            α = atan(vec[2], vec[1]) - vehicle_heading
            α = atan(sin(α), cos(α))
            L = 2.5
            computed_angle = atan(2 * L * sin(α) / lookahead_distance)
            max_steering_angle = 0.5* atan(2 * L * sin(α) / lookahead_distance)  # adjust as needed
            steering_angle = clamp(computed_angle, -max_steering_angle, max_steering_angle)
            return p, steering_angle
        end
    end
    p = [itp_x(t[end]), itp_y(t[end])]
    return p, 0.0
end

function plan_and_control(map::Dict{Int, VehicleSim.RoadSegment}, route::Vector{Int}, vehicle_pos::Vector{Float64}, vehicle_heading::Float64; lookahead_distance::Float64 = 5.0)
    itp_x, itp_y, t = plan_trajectory(map, route)
    lookahead_point, steering_command = pure_pursuit_control(vehicle_pos, vehicle_heading, itp_x, itp_y, t, lookahead_distance)
    return (lookahead_point, steering_command)
end

end  # module MotionPlanning
