module Localization

using VehicleSim
using LinearAlgebra

export EKFState, h_imu, jac_h_imu, ekf_predict, ekf_update_gps, ekf_update_imu, fuse_states

# -------------------------------
# EKF State Structure
# -------------------------------
struct EKFState
    x::Vector{Float64}  # 13-dimensional state vector:
                        # [position (3); quaternion (4); velocity (3); angular velocity (3)]
    P::Matrix{Float64}  # 13×13 covariance matrix
end

# -------------------------------
# Process Model Prediction Step
# -------------------------------
function ekf_predict(state::EKFState, dt::Float64)
    # f(x, dt) and Jac_x_f(x, dt) are assumed defined in VehicleSim/measurements.jl.
    x_pred = VehicleSim.f(state.x, dt)
    Jf = VehicleSim.Jac_x_f(state.x, dt)
    Q = 0.01 * Matrix{Float64}(I, 13, 13)   # process noise covariance (tune as needed)
    P_pred = Jf * state.P * Jf' + Q
    return EKFState(x_pred, P_pred)
end

# -------------------------------
# IMU Measurement Model and Jacobian
# -------------------------------
function h_imu(x::Vector{Float64})
    # Transforms the vehicle’s body–frame velocities into the IMU frame.
    T_body_imu = VehicleSim.get_imu_transform()
    T_imu_body = VehicleSim.invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3]
    p = T_imu_body[1:3, end]
    v_body = x[8:10]
    ω_body = x[11:13]
    ω_imu = R * ω_body
    # Compute linear velocity: v_imu = R * v_body + cross(p, ω_imu)
    v_imu = R * v_body + cross(p, ω_imu)
    return vcat(v_imu, ω_imu)
end

function jac_h_imu(x::Vector{Float64})
    # Computes the 6×13 Jacobian of h_imu at state x.
    T_body_imu = VehicleSim.get_imu_transform()
    T_imu_body = VehicleSim.invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3]
    p = T_imu_body[1:3, end]
    J = zeros(6, 13)
    # Derivative of v_imu with respect to v_body (columns 8:10)
    J[1:3, 8:10] .= R
    # Derivative of v_imu with respect to ω_body (columns 11:13)
    J[1:3, 11:13] .= skew(p) * R
    # Derivative of ω_imu with respect to ω_body (columns 11:13)
    J[4:6, 11:13] .= R
    return J
end

# Helper: Produce a skew-symmetric matrix for v × .
function skew(v::AbstractVector)
    return [ 0.0    -v[3]   v[2];
             v[3]    0.0   -v[1];
            -v[2]   v[1]    0.0 ]
end

# -------------------------------
# EKF Update for GPS
# -------------------------------
function ekf_update_gps(state::EKFState, z_gps, R_gps)
    # Uses h_gps and Jac_h_gps defined in VehicleSim/measurements.jl.
    z_pred = VehicleSim.h_gps(state.x)
    y = z_gps - z_pred
    H = VehicleSim.Jac_h_gps(state.x)
    S = H * state.P * H' + R_gps
    K = state.P * H' * inv(S)
    x_new = state.x + K * y
    I13 = Matrix{Float64}(I, 13, 13)
    P_new = (I13 - K * H) * state.P
    return EKFState(x_new, P_new)
end

# -------------------------------
# EKF Update for IMU
# -------------------------------
function ekf_update_imu(state::EKFState, z_imu, R_imu)
    # Uses the IMU measurement model (h_imu) and its Jacobian (jac_h_imu)
    z_pred = h_imu(state.x)
    y = z_imu - z_pred
    H = jac_h_imu(state.x)
    S = H * state.P * H' + R_imu
    K = state.P * H' * inv(S)
    x_new = state.x + K * y
    I13 = Matrix{Float64}(I, 13, 13)
    P_new = (I13 - K * H) * state.P
    return EKFState(x_new, P_new)
end

# -------------------------------
# Fusion Function: Selective Fusion
# -------------------------------
function fuse_states(ekf_gps::EKFState, ekf_imu::EKFState)
    # Fuse only the fields that each branch is responsible for.
    # Let the GPS branch govern position and orientation (fields 1:7)
    # and the IMU branch govern the motion (velocity and angular velocity: fields 8:13).
    x_fused = vcat(ekf_gps.x[1:7], ekf_imu.x[8:13])
    # For the covariance, construct a block-diagonal matrix.
    P_fused = zeros(13, 13)
    P_fused[1:7, 1:7] = ekf_gps.P[1:7, 1:7]
    P_fused[8:13, 8:13] = ekf_imu.P[8:13, 8:13]
    return EKFState(x_fused, P_fused)
end

end  # module Localization
