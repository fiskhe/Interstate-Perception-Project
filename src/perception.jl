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

    # l_pt = points[left_pt]
    # # x -> 1/3
    # n = l_pt[1]
    # d = l_pt[3]
    # row = []
    # push!(row, (d*R[1][1] - n*R[3][1])/d^2) # x
    # push!(row, (d*R[1][2] - n*R[3][2])/d^2) # y
    # push!(row, ((d/2)*(
    # partial_theta = [-sin() -cos()
    #                  cos()  -sin()]
    # f = [R[1][1] R[1][2]]*partial_theta*[l,w])
end

function obj_state_next()
    # x_k_forecast + kalman_gain * (cam_meas - h(x_k_forecast))
end

function obj_state_forecast()
    # dynamics!
end

function kalman_gain()
    # p_forecast * J_h(x_k_forecast).T * (J_h(x_k_forecast) * P_f * J_h(x_k_forecast).T + noise_R)^-1
end

# p is covariance
function p_k()
    # (I - kalman_gain*J_h(x_forecast))*p_forecast
end

function p_forecast()
    # J_dynamics(prev_state)*p_prev*J_dynamics(prev_state).T + noise_Q
end
