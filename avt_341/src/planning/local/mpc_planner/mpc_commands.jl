module MPC

using NLOptControl
using JuMP
using Ipopt

global current_time = 0.0
global longvel = 0.0
global latvel = 0.0
global steer_angle = 0.0
global yaw = 0.0
global x_veh = 0.0
global y_veh = 0.0
global yawrate = 0.0
global longacc = 0.0
global cmdLeaderSpeed = 0.0
global follower_status = false
global numobs = 0
global obstacle_size_meters = 0.0
global obs_radius = 0.0
global obstacles = Float64[]
global segmentation = Float32[]
global numSegCells = 0
global count = 0
global goal = [0. 0.]
global desiredHeading = 0.0
global speedSetpoint = 0.0
global cmdSpeedSetpoint = 0.0
global est_sink = 0.0
global new_sinkage_available = false
global linearSolverId = "ma27"

global n=0
global XL=0
global XU=0
global CL=0
global dx=0
global x=0
global y=0
global ux=0
global psi=0
global sr=0
global jx=0
global timeSeq=0
global Robs=0
global Xobs=0
global Yobs=0
global cellX0=0
global cellY0=0
global cellZ0=0
global obs_con=0
global obj=0
global path_prev=0

global terrainSlope=0.0
global terrainRMS=0.0

global mpc_path = Float64[]
global mpc_speed = Float64[]
global mpc_steering = Float64[]
global mpc_heading = Float64[]

global skipCount = 1
global solutionFound = false
global slopeLimited = false

# ---------- (START) PARAMETER SETTERS ----------
# Note: Must be called before Setup()
function SetTerrainSlope(terrain_slope::Float64)
	global terrainSlope = terrain_slope
end

function SetTerrainRMS(terrain_rms::Float64)
	global terrainRMS = terrain_rms
end

function SetSlopeThreshold(slope_threshold::Float64)
	global slopeThreshold = slope_threshold
end

function SetRMSThreshold(rms_threshold::Float64)
	global rmsThreshold = rms_threshold
end

function SetSteeringAngleMin(sa_min_in::Float64)
	global sa_min = sa_min_in
end

function SetSteeringAngleMax(sa_max_in::Float64)
	global sa_max = sa_max_in
end

function SetSteeringRateMin(sr_min_in::Float64)
	global sr_min = sr_min_in
end

function SetSteeringRateMax(sr_max_in::Float64)
	global sr_max = sr_max_in
end

function SetSpeedAroundLargeSlopesAndRMS(speed_around_large_slopes_and_rms::Float64)
	global speedAroundLargeSlopesAndRMS = speed_around_large_slopes_and_rms
end

function SetTireModel(tire_model::String)
	global tireModel = tire_model
end

function SetNumColPoints(col_pts::Int32)
	global numColPoints = col_pts
end

function SetPredictionTimeHorizon(t_horizon::Float64)
	global predictionTimeHorizon = t_horizon
end

function SetMaxNumObs(num_obs::Int32)
	global maxNumObs = num_obs
	global Robs = fill(1.0, maxNumObs)
	global Xobs = fill(1000.0, maxNumObs)
	global Yobs = fill(0.0, maxNumObs)
end

function SetMaxNumSeg(num_seg::Int32)
	global maxNumSeg = num_seg
end

function SetSigma(sig::Float64)
	global sigma = sig
end

function SetMinSpeed(min_speed::Float64)
	global minSpeed = min_speed
end

function SetMaxSpeed(max_speed::Float64)
	global maxSpeed = max_speed
end

function SetUseHardConstraints(use_constraints::Int32)
	global useHardConstraints = Bool(use_constraints)
end

function SetUseSegmentation(use_segmentation::Int32)
	global useSegmentation = Bool(use_segmentation)
end

function SetWDistanceToObstacles(w_dist_obs::Float64)
	global w_distanceToObstacles = w_dist_obs
end

function SetWDistanceToGoal(w_dist_goal::Float64)
	global w_distanceToGoal = w_dist_goal
end

function SetWDeviationInYaw(w_dev_yaw::Float64)
	global w_deviationInYaw = w_dev_yaw
end

function SetWYawAccel(w_yaw_accel::Float64)
	global w_yawAccel = w_yaw_accel
end

function SetWTraversabilityCost(w_traversability_cost::Float64)
	global w_traversabilityCost = w_traversability_cost
end

function SetWFinalSpeed(w_final_speed::Float64)
	global w_finalSpeed = w_final_speed
end

function SetSafetyMargin(marigin::Float64)
	global safetyMargin = marigin
end

function SetGridResolution(res::Float64)
	global grid_resolution = res
end

function SetFrontAngleGoal(angle_front::Float64)
	global frontAngleGoal = angle_front
end

function SetFrontAngleObstacle(angle_obs::Float64)
	global frontAngleObstacle = angle_obs
end

function SetTerrainAdaptive(use_adaptive::Int32)
	global adaptive = Bool(use_adaptive)
end

# function SetVehFrontAxleDist(front_axle_dist::Float64)
# 	global la = front_axle_dist
# end

function SetFrontAngleSeg(angle_seg::Float64)
	global frontAngleSegmentation = angle_seg
end

# ---------- (END) PARAMETER SETTERS ----------


function SetState(veh_data::Vector{Float64})
	#println("Received vehicle data: [",veh_data[1], ", ", veh_data[2], ", ", veh_data[3], ", ", veh_data[4], ", ", veh_data[5], ", ", veh_data[6], ", ", veh_data[7], ", ", veh_data[8], ", ", veh_data[9], ", ", veh_data[10], ", ", veh_data[11], "]")
	global current_time = veh_data[1]
	global yaw = veh_data[7]
	global x_veh = veh_data[2] + la*cos(yaw) # x position of front axle
	global y_veh = veh_data[3] + la*sin(yaw)# y position of front axle
	global longvel = veh_data[4]
	global latvel = veh_data[5]
	global steer_angle = veh_data[6]
	global yawrate = veh_data[8]
	global longacc = veh_data[9]
end

function SetObstacles(obs::Vector{Float64})
	global obstacles = obs
	global numobs = Int(length(obstacles)/3)

	if numobs > maxNumObs
		println("Number of obstacles exceeds limit. Consider increasing maxNumObs.")
	end

	for i=1:numobs
		JuMP.setValue(obs_r[i], 1.414*obstacles[3*i]/2.)
		JuMP.setValue(Xobs_0[i], obstacles[3*i-2])
		JuMP.setValue(Yobs_0[i], obstacles[3*i-1])
	end
	for i=numobs+1:maxNumObs
		JuMP.setValue(obs_r[i], Robs[i])
		JuMP.setValue(Xobs_0[i], Xobs[i])
		JuMP.setValue(Yobs_0[i], Yobs[i])
	end
end

function SetSegmentation(seg::Vector{Float64}, res::Float64)
	global segmentation = seg
	global numSegCells = Int(length(segmentation)/3)
	global gridResolution = res
	global sigma = 1.414214*gridResolution

	if !useSegmentation
		return
	end

	if numSegCells > maxNumSeg
		println("Number of segmentation cells exceeds limit [",numSegCells,">",maxNumSeg,"]. Consider increasing maxNumSeg.")
		return
	end

	if useSegmentation
		for i=1:numSegCells
			JuMP.setValue(cellX[i], segmentation[3*(i-1)+1])
			JuMP.setValue(cellY[i], segmentation[3*(i-1)+2])
			JuMP.setValue(cellZ[i], segmentation[3*(i-1)+3])
		end
		for i=numSegCells+1:maxNumSeg
			JuMP.setValue(cellX[i], cellX0)
			JuMP.setValue(cellY[i], cellY0)
			JuMP.setValue(cellZ[i], cellZ0)
		end
	end
end

function SetGoalPoint(x::Float64, y::Float64)
	global goal = [x, y]
end

function SetHeading(theta::Float64)
	global desiredHeading = theta
end

function SetSpeedSetpoint(ss::Float64)
	global speedSetpoint = ss
	global cmdSpeedSetpoint = ss
end

function SetSinkage(sinkage::Float64)
	global est_sink = sinkage
	global new_sinkage_available = true
end

function SetLinearSolver(solver::String)
    global linearSolverId = solver
end

function SetLeaderSpeed(speed::Float64)
    global cmdLeaderSpeed = speed
end

function SetFollowerStatus(status::Bool)
    global follower_status = status
end

function GetPath()
	return mpc_path
end

function GetHeading()
    return mpc_heading
end

function GetSpeed()
	num_path_points = size(mpc_path)[1]
	if skipCount < num_path_points - 5
		return mpc_speed[5+ skipCount]
	end
	return 0.0
end

function GetFinalSpeed()
	num_path_points = size(mpc_path)[1]
	return mpc_speed[num_path_points]
end

function GetSteering()
	num_path_points = size(mpc_path)[1]
	if skipCount < num_path_points - 2
		return mpc_steering[2 + skipCount]
	end
	return 0.0
end

function GetSlopeLimited()
	return slopeLimited
end

function GetObjectiveValue()
	return n.r.ocp.objVal
end

function Setup()
	global safetyMargin
	global useSegmentation
	global maxNumSeg
	global maxNumObs
	global mpc_path
	global useHardConstraints
	global sigma
	global w_distanceToObstacles
	global w_distanceToGoal
	global w_deviationInYaw
	global w_yawAccel
	global w_traversability
	global w_finalSpeed
	global cmdLeaderSpeed
	global distanceToGoal
	global distanceToObstacles
	global deviationInYaw
	global yawAccel
	global deviationFromDesiredFinalSpeed
	global traversabilityCost
	global leader_speed
	global beta
	global distanceToGoalAlongPath
	
	global mpc_path = Array{Float64}(undef, numColPoints+1, 2)
	global mpc_speed = Array{Float64}(undef, numColPoints+1, 1)
	global mpc_steering = Array{Float64}(undef, numColPoints+1, 1)
	global mpc_heading = Array{Float64}(undef, numColPoints+1, 1)

	global speedSetpoint = maxSpeed
	global cmdSpeedSetpoint = maxSpeed
	global obstacle_size_meters = grid_resolution
	global obs_radius = 1.414*obstacle_size_meters/2.0

	global n, obs_r, Xobs_0, Yobs_0, cellX, cellY, cellZ, g1, g2, desiredYaw, n_f, n_r

	XL = [NaN, NaN, NaN, NaN, psi_min, sa_min, minSpeed, -10.0]
	XU = [NaN, NaN, NaN, NaN, psi_max, sa_max, maxSpeed,  10.0]
	CL=[jx_min, sr_min]
	CU=[jx_max, sr_max]

	n = define(numStates=8,numControls=2,X0=[x_veh, y_veh, latvel, yawrate, yaw, steer_angle, longvel, longacc],XF=[NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN], XL=XL, XU=XU, CL=CL,CU=CU);

	# n.s.ocp.solver.settings[:max_cpu_time] = 10.0
	# n.s.ocp.solver.settings[:max_iter] = 3000

	defineMPC!(n;fixedTp=true,predictX0=false,tex=0.1,maxSim=1000000000)
	states!(n,[:x,:y,:v,:r,:psi,:sa,:ux,:ax];descriptions=["x(t)","y(t)","v(t)","r(t)","psi(t)","sa(t)","ux(t)","ax(t)"]);
	controls!(n,[:jx,:sr];descriptions=["jx(t)","sr(t)"]);

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

	x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2];sa = n.r.ocp.x[:,6];ux = n.r.ocp.x[:,7];psi = n.r.ocp.x[:,5];sr = n.r.ocp.u[:,2];jx = n.r.ocp.u[:,1]# pointers to JuMP variables
	timeSeq = n.ocp.tV[:,1];

	global Robs = fill(1.0, maxNumObs)
	global Xobs = fill(1000.0, maxNumObs)
	global Yobs = fill(0.0, maxNumObs)
	@NLparameter(n.ocp.mdl, obs_r[i=1:maxNumObs] == Robs[i]);
	@NLparameter(n.ocp.mdl, Xobs_0[i=1:maxNumObs] == Xobs[i]);
	@NLparameter(n.ocp.mdl, Yobs_0[i=1:maxNumObs] == Yobs[i]);
	if useSegmentation
		global cellX0 = 0.
		global cellY0 = 0.
		global cellZ0 = 0.
		@NLparameter(n.ocp.mdl, cellX[i=1:maxNumSeg] == cellX0)
		@NLparameter(n.ocp.mdl, cellY[i=1:maxNumSeg] == cellY0)
		@NLparameter(n.ocp.mdl, cellZ[i=1:maxNumSeg] == cellZ0)
	end
	@NLparameter(n.ocp.mdl, g1 == 50.0);
	@NLparameter(n.ocp.mdl, g2 == 0.0);
	@NLparameter(n.ocp.mdl, rhoKS == 2.5 * obstacle_size_meters)
	if useHardConstraints
		ksAggregation = @NLexpression(n.ocp.mdl, sum(exp(rhoKS * (- (x[i]-Xobs_0[j])^2.0 - (y[i]-Yobs_0[j])^2.0 + (obs_r[j] + safetyMargin)^2.0)) for j=1:maxNumObs for i=2:n.ocp.state.pts))
		obs_con = @NLconstraint(n.ocp.mdl, ksAggregation <= 1.0)
		newConstraint!(n,obs_con,:obs_con)
	end
	@NLparameter(n.ocp.mdl, desiredYaw == 0.0); #desired yaw angle
	@NLparameter(n.ocp.mdl, leader_speed == 0.0);

	# Traditional endpoint-based distance to goal
	distanceToGoal=@NLexpression(n.ocp.mdl,(((x[end]-g1)^2+(y[end]-g2)^2)/((x[1]-g1)^2+(y[1]-g2)^2)))
	
	# Smooth minimum distance to goal along the entire path
	# Uses exponential sum for a differentiable measure of path-goal proximity
	@NLparameter(n.ocp.mdl, beta == beta)
	distanceToGoalAlongPath = @NLexpression(n.ocp.mdl,
		-sum(exp(-beta * ((x[j]-g1)^2 + (y[j]-g2)^2)) for j=1:n.ocp.state.pts)
	)
	
	# distanceToObstacles = @NLexpression(n.ocp.mdl,sum(1/((x[j]-Xobs_0[i])^2+(y[j]-Yobs_0[i])^2+0.1) for i=1:maxNumObs for j=2:n.ocp.state.pts))
	# distanceToObstacles = @NLexpression(n.ocp.mdl,sum((tanh(-1.3*((x[j] - Xobs_0[i])^2/(obs_r[i] + safetyMargin)^2 +(y[j] - Yobs_0[i])^2/(obs_r[i] + safetyMargin)^2)) + 1)/2 for i=1:maxNumObs for j=2:n.ocp.state.pts))
	distanceToObstacles = @NLexpression(n.ocp.mdl,sum((exp(-((x[j] - Xobs_0[i])^2/(obs_r[i] + safetyMargin)^2 +(y[j] - Yobs_0[i])^2/(obs_r[i] + safetyMargin)^2)) + 1)/2 for i=1:maxNumObs for j=2:n.ocp.state.pts))

	deviationInYaw = @NLexpression(n.ocp.mdl, (cos(psi[2])-cos(desiredYaw))^2+(sin(psi[2])-sin(desiredYaw))^2)
	yawAccel = @NLexpression(n.ocp.mdl, sum((ux[i] * sr[i])^2 for i=2:n.ocp.state.pts))
	deviationFromDesiredFinalSpeed = @NLexpression(n.ocp.mdl, (ux[end] - leader_speed)^2)
	if useSegmentation
		traversabilityCost = @NLexpression(n.ocp.mdl, sum(cellZ[i]*exp(-((x[j] - cellX[i])^2/(sigma)^2 +(y[j] - cellY[i])^2/(sigma)^2)) for i=1:maxNumSeg for j=2:n.ocp.state.pts))
	end
	obj = integrate!(n,:( 10.0*sr[j]^2. + 0.01*jx[j]^2.))
	@NLobjective(n.ocp.mdl, Min, obj + w_distanceToGoal*distanceToGoal + w_distanceToObstacles*distanceToObstacles + w_deviationInYaw*deviationInYaw + w_yawAccel*yawAccel)
	# @NLobjective(n.ocp.mdl, Min, obj+100.0 * (distanceToGoal+1))
	if useSegmentation
		@NLobjective(n.ocp.mdl, Min, obj + w_distanceToGoal*distanceToGoal + w_distanceToObstacles*distanceToObstacles + w_deviationInYaw*deviationInYaw + w_yawAccel*yawAccel + w_traversabilityCost*traversabilityCost)
	else
		@NLobjective(n.ocp.mdl, Min, obj + w_distanceToGoal*distanceToGoal + w_distanceToObstacles*distanceToObstacles + w_deviationInYaw*deviationInYaw + w_yawAccel*yawAccel)
	end
	n.s.ocp.save = false

	JuMP.setsolver(n.ocp.mdl, Ipopt.IpoptSolver(;
		linear_solver = linearSolverId,
		max_iter = 2000,
		print_level = 0,
		warm_start_init_point = "no",
	))

	if n.s.mpc.shiftX0
		for st in 1:n.ocp.state.num
			if n.ocp.X0[st] < n.ocp.XL[st]
				n.ocp.X0[st] = n.ocp.XL[st]
			end
			if n.ocp.X0[st] > n.ocp.XU[st]
				n.ocp.X0[st] = n.ocp.XU[st]
			end
		end
	end
	for st in 1:n.ocp.state.num
		JuMP.setRHS(n.r.ocp.x0Con[st],n.ocp.X0[st])
	end

	# println("Goal: ",JuMP.getvalue(g1)," ",JuMP.getvalue(g2))
	optimize!(n)
	# println("Initialization status: ",n.r.ocp.status)
	# println("Setup done. Type 'q' to quit.")

	JuMP.setsolver(n.ocp.mdl, Ipopt.IpoptSolver(;
		linear_solver = linearSolverId,
		max_iter = 200,
		max_cpu_time = 0.2,
		print_level = 0,
		warm_start_init_point = "yes",
		# mu_strategy = "adaptive",
		tol = 2e-1,
		dual_inf_tol = 1.,
		constr_viol_tol = 3e-1,
		compl_inf_tol = 3e-1,
		acceptable_tol = 7e-2,
		acceptable_constr_viol_tol = 0.01,
		acceptable_dual_inf_tol = 1e10,
		acceptable_compl_inf_tol = 0.01
	))

	# n.s.ocp.solver.settings[:max_cpu_time] = 1.0
	# n.s.ocp.solver.settings[:max_iter] = 200
end

function Plan()
	global mpc_path, mpc_speed, mpc_steering, mpc_heading, solutionFound, skipCount, path_prev, numobs, obstacles, speedSetpoint, cmdSpeedSetpoint, slopeLimited
	global follower_status
	global cmdLeaderSpeed
	global distanceToGoal
	global distanceToObstacles
	global deviationInYaw
	global yawAccel
	global deviationFromDesiredFinalSpeed
	global traversabilityCost
	global distanceToGoalAlongPath

	# stop calculating if previous path already reached the goal
	if false && path_prev != 0 && maximum(sqrt.((path_prev[:,1] .- goal[1]).^2. .+ (path_prev[:,2] .- goal[2]).^2.) .< 2.0)
		mpc_path = path_prev
	else
		#NOTE we may want to ignore eerything except for position, heading, and longitudinal speed, if MPC will not control the vehicle
		X0 = [x_veh, y_veh, latvel, yawrate, yaw, steer_angle, longvel, longacc]
		# t0 = current_time

		if adaptive
			####### Change Sinkage Exponent ###########

			if new_sinkage_available
				global est_sink
				global n_f
				global n_r
				@NLparameter(n.ocp.mdl, n_f == est_sink)
				@NLparameter(n.ocp.mdl, n_r == est_sink)
				new_sinkage_available = false
			end

			####### END -- Change Sinkage Exponent ###########
		end

		JuMP.setValue(g1, goal[1])
		JuMP.setValue(g2, goal[2])
		JuMP.setValue(desiredYaw, desiredHeading)
		push!(n.r.ip.X0p,X0)
		n.ocp.X0 = X0
		push!(n.r.ocp.X0, n.ocp.X0)
		# setvalue(n.ocp.t0, copy(t0))

		# modify speed setpoint if there is significant slope or rms
		if terrainSlope > slopeThreshold || terrainRMS > rmsThreshold
			speedSetpoint = speedAroundLargeSlopesAndRMS
			slopeLimited = true
		elseif speedSetpoint != cmdSpeedSetpoint
			speedSetpoint = cmdSpeedSetpoint
			slopeLimited = false
		else
			slopeLimited = false	# For when the limit is the same as commanded
		end

		#set the new speed limit
		n.ocp.XU[7]=speedSetpoint
		for i=1:n.ocp.state.pts
			setupperbound(n.r.ocp.xUnscaled[i,7],n.ocp.XU[7])
		end
		
		# Update objective function based on follower status and goal proximity
		# Check if goal is within prediction horizon
		goal_dist = sqrt((goal[1] - x_veh)^2 + (goal[2] - y_veh)^2)
		horizon_dist = speedSetpoint * predictionTimeHorizon
		if follower_status && (goal_dist < horizon_dist)
			JuMP.setValue(leader_speed, cmdLeaderSpeed)
			@NLobjective(n.ocp.mdl, Min, obj + w_distanceToGoal*distanceToGoalAlongPath + w_distanceToObstacles*distanceToObstacles + w_deviationInYaw*deviationInYaw + w_yawAccel*yawAccel + w_finalSpeed*deviationFromDesiredFinalSpeed)
		else
			@NLobjective(n.ocp.mdl, Min, obj + w_distanceToGoal*distanceToGoal + w_distanceToObstacles*distanceToObstacles + w_deviationInYaw*deviationInYaw + w_yawAccel*yawAccel)
		end

		if n.s.mpc.shiftX0
			for st in 1:n.ocp.state.num
				if n.ocp.X0[st] < n.ocp.XL[st]
					n.ocp.X0[st] = n.ocp.XL[st]
				end
				if n.ocp.X0[st] > n.ocp.XU[st]
					n.ocp.X0[st] = n.ocp.XU[st]
				end
			end
		end
		for st in 1:n.ocp.state.num
			JuMP.setRHS(n.r.ocp.x0Con[st],n.ocp.X0[st])
		end

		optimize!(n)

		# println(n.r.ocp.status," (",round(1000*n.r.ocp.tSolve; digits = 1)," ms)")
		if n.r.ocp.status == :Optimal
			solutionFound = true
			@views X = n.r.ocp.X;  # Create views to avoid unnecessary copying
			mpc_path[:,1] = X[:,1] .- (la .* cos.(X[:,5]));
			mpc_path[:,2] = X[:,2] .- (la .* sin.(X[:,5]));
			mpc_speed = X[:,7]
			mpc_steering = X[:,6]
			mpc_heading = X[:,5]
			skipCount = 1
		else
			solutionFound = false
			if path_prev != 0
				mpc_path = path_prev
				skipCount = skipCount + 1
			end
		end
	end
end

end # module MPC
