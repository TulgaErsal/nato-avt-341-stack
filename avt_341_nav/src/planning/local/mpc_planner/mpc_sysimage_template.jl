"""
@file mpc_sysimage_template.jl

@brief Template for the Julia sysimage generation.

@date 10/12/2023

@author Dario Sirangelo (dsi@mpe.au.dk)
        Aarhus University (DK)
        Department of Mechanical and Production Engineering
        Section Mechatronics & Dynamics

@date 05/03/2025

@author Tulga Ersal (tersal@umich.edu)
        University of Michigan
        Department of Mechanical Engineering        
"""

using NLOptControl
using JuMP
using Ipopt
include("mpc_parameters.jl")
include("mpc_models.jl")


numColPoints = 5
predictionTimeHorizon = 2.0


function main()

	n=0;XL=0;XU=0;CL=0;goal=0;dx=0;ssm=0;lsm=0;x=0;y=0;ux=0;psi=0;sr=0;jx=0;timeSeq=0;Aobs=0;Bobs=0;Xobs=0;Yobs=0;obs_con=0;obj=0;path_prev=0

	XL = [-500, -500, NaN, NaN, psi_min, -0.855, 2.0, -10.0]
	XU = [500, 500, NaN, NaN, psi_max, 0.855, 35.0,  10.0]
	CL=[jx_min, sr_min]
	CU=[jx_max, sr_max]
	n = define(numStates=8,numControls=2,X0=[-75.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15., 0.],XF=[NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN], XL=XL, XU=XU, CL=CL,CU=CU);

	goal = [40.0, 0.0]

	n.s.ocp.solver.settings[:max_cpu_time] = 10.0
	n.s.ocp.solver.settings[:max_iter] = 3000

	defineMPC!(n;fixedTp=true,predictX0=false,tex=0.5,maxSim=1000000000)

	states!(n,[:x,:y,:v,:r,:psi,:sa,:ux,:ax];descriptions=["x(t)","y(t)","v(t)","r(t)","psi(t)","sa(t)","ux(t)","ax(t)"])
	controls!(n,[:jx,:sr];descriptions=["jx(t)","sr(t)"])
	dx=ThreeDOF_rigid(n)
	dynamics!(n,dx)
	configure!(n,N=numColPoints;(:integrationScheme=>:bkwEuler),(:tf=>predictionTimeHorizon))

	ssm = 2.7
	lsm = 1.7
	x = n.r.ocp.x[:,1];y = n.r.ocp.x[:,2];ux = n.r.ocp.x[:,7];psi = n.r.ocp.x[:,5];sr = n.r.ocp.u[:,2];jx = n.r.ocp.u[:,1]# pointers to JuMP variables
	timeSeq = n.ocp.tV[:,1]

	Aobs = fill(1.0, 3)
	Bobs =fill(1.0, 3)
	Xobs = fill(100.0, 3)
	Yobs = fill(0.0, 3)
	@NLparameter(n.ocp.mdl, obs_a[i=1:3] == Aobs[i])
	@NLparameter(n.ocp.mdl, obs_b[i=1:3] == Bobs[i])
	@NLparameter(n.ocp.mdl, Xobs_0[i=1:3] == Xobs[i])
	@NLparameter(n.ocp.mdl, Yobs_0[i=1:3] == Yobs[i])
	obs_con = @NLconstraint(n.ocp.mdl, [j=1:3,i=1:n.ocp.state.pts-1], 1 <= ((x[(i+1)]-Xobs_0[j])^2)/((obs_a[j]+ssm)^2) + ((y[(i+1)]-Yobs_0[j])^2)/((obs_b[j]+lsm)^2))
	newConstraint!(n,obs_con,:obs_con)


	obj = integrate!(n,:( 1.0*sr[j]^2 + 0.01*jx[j]^2+1.2*y[j]^2))
	@NLobjective(n.ocp.mdl, Min, obj+10.0*n.ocp.tf+(n.r.ocp.x[end,1]-goal[1])^2+(n.r.ocp.x[end,2]-goal[2])^2)

	JuMP.setValue(obs_a[1], (1.414))
	JuMP.setValue(obs_b[1], (1.414))
	JuMP.setValue(Xobs_0[1], 0)
	JuMP.setValue(Yobs_0[1], 20)

	n.ocp.X0 = [-75.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15., 0.]
	push!(n.r.ocp.X0, n.ocp.X0)
	setvalue(n.ocp.t0, copy(0.0))
	for st in 1:n.ocp.state.num
		JuMP.setRHS(n.r.ocp.x0Con[st],n.ocp.X0[st])
	end

	JuMP.setsolver(n.ocp.mdl, Ipopt.IpoptSolver(;
	linear_solver = "ma27",
	max_iter = 2000,
	print_level = 0,
	warm_start_init_point = "no"
	))

	optimize!(n);
	println(n.r.ocp.status," (",round(1000*n.r.ocp.tSolve; digits = 1)," ms)")
	path_prev = Array{Float64}(undef, numColPoints+1, 2)
	mpc_path = Array{Float64}(undef, numColPoints+1, 2)
        @views X = n.r.ocp.X;
        mpc_path[:,1] = X[:,1] .- (1.25 .* cos.(X[:,5]));
        mpc_path[:,2] = X[:,2] .- (1.25 .* sin.(X[:,5]));
	path_prev = mpc_path
	println("Finished running system image template.")

end


main()
