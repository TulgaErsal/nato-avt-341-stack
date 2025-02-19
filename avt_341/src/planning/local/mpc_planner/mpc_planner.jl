"""
@file mpc_planner.jl

@brief Plan a local trajectory using the model predictive control planner.

@date 10/12/2023

@author Tulga Ersal (tersal@umich.edu)
        University of Michigan
        Department of Mechanical Engineering
"""

module MPC

# Add packages namespaces to this module - these directives must be place AFTER
# the module definition.
using Ipopt
using JuMP
using NLOptControl

# TODO: I should really double check whether all global variables are all
# properly initialised to meaningful values from the ROS node.
global goal = [0, 0]
global t_span = 5.0
global u_min = 0.01 # 0.0
global d = 2
global l_a = 1.0
global use_terrain_adaptive = false
global est_sinkage = 0.0
global desiredHeading = 0.0

# TEMP
global yaw = 0
global x_veh = 0
global y_veh = 0
global longvel = 0
global latvel = 0
global steer_angle = 0
global yawrate = 0
global longacc = 0

#ADDED array that stores the heading trajectory (psi)
global mpc_heading = Float64[]

function Plan()
	global stop = false
	global wait
	global sm_x
	global sm_y
	global goal
	global maxNumObs
	global obstacle_size_meters = 1.0 # TODO: This must be assigned
	global obs_radius
	global mpc_path
	global useHardConstraints

	n=0;XL=0;XU=0;CL=0;dx=0;x=0;y=0;ux=0;psi=0;sr=0;jx=0;timeSeq=0;Robs=0;Xobs=0;Yobs=0;obs_con=0;obj=0;path_prev=0

	XL = [NaN, NaN, NaN, NaN, psi_min, sa_min, minSpeed, -10.0]
	XU = [NaN, NaN, NaN, NaN, psi_max, sa_max, maxSpeed,  10.0]
	CL = [jx_min, sr_min]
	CU = [jx_max, sr_max]

    global rhoKS = 1.0 # TODO: This must be assigned

	n = define(numStates=8,
               numControls=2,
               X0=[x_veh, y_veh, latvel, yawrate, yaw, steer_angle, longvel, longacc],
               XF=[NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN],
               XL=XL,
               XU=XU,
               CL=CL,
               CU=CU);

	defineMPC!(n;
               fixedTp=true,
               predictX0=false,
               tex=0.1,
               maxSim=1000000000)
	states!(n,
            [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
            descriptions=["x(t)","y(t)","v(t)","r(t)","psi(t)","sa(t)","ux(t)","ax(t)"]);

    controls!(n,
              [:jx,:sr];
              descriptions=["jx(t)","sr(t)"]);

    if adaptive
        if tireModel != "N"
            println("Terrain adaptation is turned on without setting the tire model to N. Ignoring the tire model setting and using N instead.")
        end
        @NLparameter(n.ocp.mdl, n_f == n_guess)
        @NLparameter(n.ocp.mdl, n_r == n_guess)
        @NLparameter(n.ocp.mdl, f_kappa == 0.01)
        @NLparameter(n.ocp.mdl, r_kappa == 0.01)
        dx=ThreeDOF_deformable(n, n_f, n_r, f_kappa, r_kappa)
    elseif tireModel == "P"
        dx=ThreeDOF_rigid(n)
    elseif tireModel == "L"
        dx=ThreeDOF_linear(n)
    elseif tireModel == "N"
        @NLparameter(n.ocp.mdl, n_f == n_guess)
        @NLparameter(n.ocp.mdl, n_r == n_guess)
        @NLparameter(n.ocp.mdl, f_kappa == 0.01)
        @NLparameter(n.ocp.mdl, r_kappa == 0.01)
        dx=ThreeDOF_deformable(n, n_f, n_r, f_kappa, r_kappa)
    end

    dynamics!(n,dx);
    configure!(n,N=numColPoints;(:integrationScheme=>:bkwEuler),(:tf=>predictionTimeHorizon));

	x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2];ux = n.r.ocp.x[:,7];psi = n.r.ocp.x[:,5];sr = n.r.ocp.u[:,2];jx = n.r.ocp.u[:,1]# pointers to JuMP variables
	timeSeq = n.ocp.tV[:,1];

	Robs = fill(1.0, maxNumObs)
	Xobs = fill(1000.0, maxNumObs)
	Yobs = fill(0.0, maxNumObs)
	@NLparameter(n.ocp.mdl, obs_r[i=1:maxNumObs] == Robs[i]);
	@NLparameter(n.ocp.mdl, Xobs_0[i=1:maxNumObs] == Xobs[i]);
	@NLparameter(n.ocp.mdl, Yobs_0[i=1:maxNumObs] == Yobs[i]);
	@NLparameter(n.ocp.mdl, g1 == 50.0);
	@NLparameter(n.ocp.mdl, g2 == 0.0);
	@NLparameter(n.ocp.mdl, rhoKS == 2.5 * obstacle_size_meters)
	if useHardConstraints
		ksAggregation = @NLexpression(n.ocp.mdl, sum(exp(rhoKS * (- (x[i]-Xobs_0[j])^2.0 - (y[i]-Yobs_0[j])^2.0 + (obs_r[j] + sm_x)^2.0)) for j=1:maxNumObs for i=2:n.ocp.state.pts))
		obs_con = @NLconstraint(n.ocp.mdl, ksAggregation <= 1.0)
		newConstraint!(n,obs_con,:obs_con)
	end
	
	solve!(n)
	
	global mpc_heading = [value(psi[i]) for i in 1:length(psi)]
	#@NLparameter(n.ocp.mdl, desiredYaw == 0.0); #desired yaw angle

	#distanceToGoal=@NLexpression(n.ocp.mdl,(((x[end]-g1)^2+(y[end]-g2)^2)/((x[1]-g1)^2+(y[1]-g2)^2)))
	#distanceToObstacles = @NLexpression(n.ocp.mdl,sum((tanh(-1.3*((x[j] - Xobs_0[i])^2/(obs_r[i] + sm_x)^2 +(y[j] - Yobs_0[i])^2/(obs_r[i] + sm_y)^2)) + 1)/2 for i=1:maxNumObs for j=2:n.ocp.state.pts))
	#deviationInYaw = @NLexpression(n.ocp.mdl, (cos(psi[2])-cos(desiredYaw))^2+(sin(psi[2])-sin(desiredYaw))^2)

    println(XL)
end

"""
    SetSinkage(n)

Set the estimated sinkage exponent.
"""

function GetHeading()
    global mpc_heading
    return mpc_heading
end


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
    # The (X,Y) position is referenced to the front axle!
    global x_veh = x * la * cos(yaw)
    global y_veh = y * la * sin(yaw)
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
    SetHeading(l)

Set the planner desired heading.
"""
function SetHeading(psi::Float64)
    global desiredHeading_input = psi
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
