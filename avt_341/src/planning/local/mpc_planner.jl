"""
@file avt_341_mpc_planner_node.cpp

@brief Plan a local trajectory using the model predictive control planner.
       This ROS node is a wrapper to the TulgaErsal/AVT-341-MPC planner
       through the Julia C API.

@date 03/11/2023

@author Dario Sirangelo (dsi@mpe.au.dk)
        Aarhus University (DK)
        Department of Mechanical and Production Engineering
        Section Mechatronics & Dynamics
"""

module MPC

# TODO: I should really double check whether all global variables are all
# properly initialised to meaningful values from the ROS node.
global goal = [0, 0]
global t_span = 5.0
global u_min = 0.0
global d = 2
global l_a = 1.0
global use_terrain_adaptive = false
global est_sinkage = 0.0

"""
    SetSinkage(n)

Set the estimated sinkage exponent.
"""
function SetSinkage(n::Float64)
    global est_sinkage = n
end

"""
    SetUseTerrainAdaptive(f)

Set whether or not the planner should use the terrain-adaptive formulation.
"""
function SetUseTerrainAdaptive(f::Bool)
    global use_terrain_adaptive = f
end

"""
    SetState(x, y, u, v, delta, psi, r, a)

Set the vehicle state.
"""
function SetState(x::Float64,
                  y::Float64,
                  u::Float64,
                  v::Float64,
                  delta::Float64,
                  psi::Float64,
                  r::Float64,
                  a::Float64)
    global yaw = psi
    global x_veh = x
    global y_veh = y
    global longvel = u
    global latvel = v
    global steer_angle = delta
    global yawrate = r
    global longacc = a
end

"""
    SetFrontAxlePosition(l)

Set the distance between the centre of gravity of the vehicle and the centre of
the front axle.
"""
function SetFrontAxlePosition(l::Float64)
    global l_a = l
end

"""
    SetMinimumSpeed(u)

Set the minimum allowed vehicle speed in the planner.
"""
function SetMinimumSpeed(u::Float64)
    global u_min = u
end

"""
    SetPredictionHorizon(t)

Set the MPC prediction horizon in seconds.
"""
function SetPredictionHorizon(t::Float64)
    global t_span = t
end

"""
    SetGoalPoint(t)

Set the planner goal point.
"""
function SetGoalPoint(x::Float64,
                      y::Float64)
    global goal = [x, y]
end

end