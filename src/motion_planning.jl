module MotionPlanning

using VehicleSim
using Interpolations
using LinearAlgebra

export plan_trajectory, pure_pursuit_control, plan_and_control

# extract midline points from lane boundaries
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

# Plan a trajectory using cubic spline interpolation
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

# Pure Pursuit Control
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

# Main function to plan and control
function plan_and_control(map::Dict{Int, VehicleSim.RoadSegment}, route::Vector{Int}, vehicle_pos::Vector{Float64}, vehicle_heading::Float64; lookahead_distance::Float64 = 5.0)
    itp_x, itp_y, t = plan_trajectory(map, route)
    lookahead_point, steering_command = pure_pursuit_control(vehicle_pos, vehicle_heading, itp_x, itp_y, t, lookahead_distance)
    return (lookahead_point, steering_command)
end

end  # module MotionPlanning
