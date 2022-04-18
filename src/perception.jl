using LinearAlgebra # for the I

struct ObjectState
    x::Float64
    y::Float64
    θ::Float64
    length::Float64
    width::Float64
    height::Float64
end

struct TracksMessage
    timestamp::Float64
    tracks::Dict{Int, ObjectState}
end



function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, camera_array, road)
    lines = []

    # see PinholeCamera struct line 60 in sensors.jl

    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)
        #tracks = TracksMessage(...)
        #TODO your code here
        #@replace(TRACKS, tracks)    
    end
end

function h_Jacobian(full_state, camera)
    # full_state = [x y theta l w h v \omega]
    # in sensors.jl:
    # get_camera_meas -> expected_bbox
    #                 -> get_corners
    # get_corners is in movables.jl line 98
# ∈
    # ms stands for moveables
    # you get moveables from simulator_state which is from SIM_ALL in launch files
    # wait you can look at the moveables generated in launch_perception starting from line 24
    # 
    # update_ensor with camera array is the one being used, not the piholecamera one

    (x, y, θ, l, w, h) = full_state[1:6]

    f = camera.focal_len
    R = camera.R
    t = camera.t

    A = [cos(θ)/2  -sin(θ)/2 0
         sin(θ)/2  cos(θ)/2 0
         0         0        1]
    # question: shouldnt the 2nd row sin be negative according ot notes???

    lwh = [l, w, h]
    pos = [x, y, 0]
    points = []
    # + and - lwh to get all 8 points for possible BB coords
    for i in [1, -1]
        for j in [1, -1]
            for k in [0, 1]
                # rf_cam = R*(pos + A*Diagonal([i,j, k])*lwh) + t
                push!(points, R*(pos + A*Diagonal([i, j, k])*lwh) + t)
            end
        end
    end

    (left, top, right, bottom) = (Inf, Inf, -Inf, -Inf)
    (left_pt, top_pt, right_pt, bottom_pt) = (0, 0, 0, 0)
    counter = 1
    # look at expected_bbox in sensors.jl, line 117
    for rf_cam in points
        # px = max(min(camera.focal_len * pt[1] / (pt[3] * camera.sx), 1.0), -1.0)
        cam_x = max(min(f*rf_cam[1]/(rf_cam[3]*camera.sx), 1.0), -1.0)
        cam_y = max(min(f*rf_cam[2]/(rf_cam[3]*camera.sy), 1.0), -1.0)
        if cam_x > right
            right = cam_x
            right_pt = counter
        end
        if cam_y < top
            top = cam_y
            top_pt = counter
        end
        if cam_x < left
            left = cam_x
            left_pt = counter
        end
        if cam_y > bottom
            bottom = cam_y
            bottom_pt = counter
        end
        counter += 1
    end

    return [left top right bottom] # if just returning h output

    l_pt = points[left_pt]
    # x -> 1/3
    n = l_pt[1]
    d = l_pt[3]
    row = []
    push!(row, ) # x
    push!(row, ) # y
end

function dh_dx(o, u, n, d, R)
    u = 3
    (d*R[o][1] - n*R[u][1]) / d^2
end

function dh_dy(o, u, n, d, R)
    (d*R[o][2] - n*R[u][2]) / d^2
end

function dh_dtheta(o, u, n, d, R)
    #over under numerator denominator R

    M = [-sin() -cos()
         cos()  -sin()]

    d_high = [R[o][1] R[o][2]] * M * [l,w]
    d_low = [R[u][1] R[u][2]] * M * [l,w]

    (d/2 * d_high - n/2 * d_low) / d^2
end

function dh_dl(o, u, n, d, R)
    d_high = R[o][1]*cos() + R[o][2]sin()
    d_low = R[u][1]*cos() + R[u][2]*sin()
    (d/2 * d_high - n/2 * d_low) / d^2
end

function dh_dw(o, u, n, d, R)
    d_high = -R[o][1]*sin() + R[o][2]cos()
    d_low = -R[u][1]*sin() + R[u][2]*cos()
    (d/2 * d_high - n/2 * d_low) / d^2
end

function dh_dh(o, u, n, d, R)
    R[o][3] / d
end

function obj_state_next(obj_state, cam_meas)
    # x_k_forecast + kalman_gain * (cam_meas - h(x_k_forecast))
    x_f = obj_state_forecast(obj_state, 0.1)
end

function obj_state_forecast(obj_state, Δ)
    # dynamics!
    θ = obj_state[3]
    v = obj_state[7]
    ω = obj_state[8]
    obj_state[1:3] +=  [Δ * cos(θ) * v, Δ * cos(θ) * v, Δ * ω]
    return obj_state
end

function J_dynamics_forecast(obj_state, Δ)
    # jacobian of above.. should be 8x8 (dimensions of object state)
end

function kalman_gain()
    # p_forecast * J_h(x_k_forecast).T * (J_h(x_k_forecast) * P_f * J_h(x_k_forecast).T + noise_R)^-1
end

# p is covariance
function p_k()
    # (I - kalman_gain*J_h(x_forecast))*p_forecast
end

function p_forecast(prev_state)
    # J_dynamics(prev_state)*p_prev*J_dynamics(prev_state).T + noise_Q
end
