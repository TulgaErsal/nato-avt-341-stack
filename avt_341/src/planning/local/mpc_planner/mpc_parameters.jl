"""
@file mpc_parameters.jl

@brief Parameters for the model predictive control planner.

@date 10/12/2023

@author Tulga Ersal (tersal@umich.edu)
        University of Michigan
        Department of Mechanical Engineering
"""

tireModel = "L" #L for linear, P for Pacejka, N for neural network for deformable terrain

numColPoints = 10 #number of collocation points to use in MPC
predictionTimeHorizon = 2.0 #MPC predition time horizon [s]
maxNumObs = 500 #maximum number of obstacles allowed in MPC
minSpeed = 0.5 #minimum allowed vehicle speed [m/s]
maxSpeed = 3.5 #maximum allowed vehicle speed [m/s]

stopOnMaxSolveTime = true # Stop the vehicle when maximum solve time is reached

useHardConstraints = false #whether to include obstacles as hard constraints in addition to soft constraints
w_distanceToObstacles = 5. #weight on distanceToObstacles in the objective function
w_distanceToGoal = 100. #weight on distanceToGoal in the objective function
w_deviationInYaw = 1. #weight on deviationInYaw in the objective function

safetyMargin = 0.0 #safety margin [m] around obstacles
grid_resolution = 0.25 #resulution of the occupancy grid [m]

frontAngleGoal = 90 * π / 180 #the angle measured from the vehicle heading beyond which the goal point is considered not in front of the vehicle
frontAngleObstacle = 90 * π / 180 #the angle measured from the vehicle heading beyond which an obstacle is considered not in front of the vehicle

#whether to use the terrain adaptive formulation or not
adaptive = false

# initial guess for sinkage exponent n in the deformable tire model
n_guess = 0.7

# define model parameters

# Longitudinal Pacejka model parameters
PCX1 = 1.18358               #Shape factor Cfx for longitudinal force
PDX1 = 1.04493              #Longitudinal friction Mux at Fznom
PDX2 = -0.0966588            #Variation of friction Mux with load
PDX3 = 1.1979          #Variation of friction Mux with camber
PEX1 = -0.320013             #Longitudinal curvature Efx at Fznom
PEX2 = -0.476432               #Variation of curvature Efx with load
PEX3 = 0.333948             #Variation of curvature Efx with load squared
PEX4 = -2.22797          #Factor in curvature Efx while driving
PKX1 = 11.0862               #Longitudinal slip stiffness Kfx/Fz at Fznom
PKX2 = -0.746427               #Variation of slip stiffness Kfx/Fz with load
PKX3 = 0.0359293              #Exponent in slip stiffness Kfx/Fz with load
PHX1 = 0.002233         #Horizontal shift Shx at Fznom
PHX2 = -0.00226917          #Variation of shift Shx with load
PVX1 = -0.0386225         #Vertical shift Svx/Fz at Fznom
PVX2 = -0.000258126         #Variation of shift Svx/Fz with load
RBX1 = 100.0 #Slope factor for combined slip Fx reduction
RBX2 = 15.1701 #Variation of slope Fx reduction with kappa
RBX3 = 0 #Influence of camber on stiffness for Fx combined
RCX1 = 0.598864 #Shape factor for combined slip Fx reduction
REX1 = 0.794343 #Curvature factor of combined Fx
REX2 = 0.102829 #Curvature factor of combined Fx with load
RHX1 = 0.00165907 #Shift factor for combined slip Fx reduction
PPX1 = 0.0 #Linear pressure effect on slip stiffness
PPX2 = 0.0 #Quadratic pressure effect on slip stiffness
PPX3 = 0.0 #Linear pressure effect on longitudinal friction
PPX4 = 0.0 #Quadratic pressure effect on longitudinal friction
PTX1 = 0.0   #Relaxation length SigKap0 / Fz at Fznom
PTX2 = 0.0   #Variation of SigKap0 / Fz with load
PTX3 = 0.0   #Variation of SigKap0 / Fz with exponent of load

# Lateral_Pacejka model parameters
PCY1 = 1.437525               #Shape factor Cfy for lateral forces
PDY1 = -0.9277398              #Lateral friction Muy
PDY2 = 0.0629010            #Variation of friction Muy with load
PDY3 = -1.3139633              #Variation of friction Muy with squared camber
PEY1 = 0.1210240              #Lateral curvature Efy at Fznom
PEY2 = -0.0434791            #Variation of curvature Efy with load
PEY3 = -0.0896877              #Zero order camber dependency of curvature Efy
PEY4 = 35.937783               #Variation of curvature Efy with camber
PEY5 = 0.0 #Camber curvature Efc
PKY1 = -10.601958              #Maximum value of stiffness Kfy/Fznom
PKY2 =  2.4674040               #Load at which Kfy reaches maximum value
PKY3 = 0.6617869             #Variation of Kfy/Fznom with camber
PKY4 = 0.0 #Peak stiffness variation with camber squared
PKY5 = 0.0 #Lateral stiffness dependency with camber
PKY6 = 0.0 #Camber stiffness factor
PKY7 = 0.0 #Load dependency of camber stiffness factor
PHY1 = -0.0014867            #Horizontal shift Shy at Fznom
PHY2 = 5.4986956e-04            #Variation of shift Shy with load
PHY3 = 0.1255658              #Variation of shift Shy with camber
PVY1 = 0.0053318             #Vertical shift in Svy/Fz at Fznom
PVY2 = -0.0047437            #Variation of shift Svy/Fz with load
PVY3 = 0.0630867           #Variation of shift Svy/Fz with camber
PVY4 = -0.2980209            #Variation of shift Svy/Fz with camber and load

RBY1 = 8.67134               #Slope factor for combined Fy reduction
RBY2 = 10.5608               #Variation of slope Fy reduction with alpha
RBY3 = 0.0349066         #Shift term for alpha in slope Fy reduction
RBY4 = 0 #Influence of camber on stiffness of Fy combined
RCY1 = 1.14123                  #Shape factor for combined Fy reduction
REY1 = 0.497453              #Curvature factor of combined Fy
REY2 = -0.00951908          #Curvature factor of combined Fy with load
RHY1 = 0.00223554             #Shift factor for combined Fy reduction
RHY2 = 0.0154843          #Shift factor for combined Fy reduction with load
RVY1 = 0.0           #Kappa induced side force Svyk/Muy*Fz at Fznom
RVY2 = 0.            #Variation of Svyk/Muy*Fz with load
RVY3 = 1.49012e-08            #Variation of Svyk/Muy*Fz with camber
RVY4 = 0.0            #Variation of Svyk/Muy*Fz with alpha
RVY5 = 0.0                  #Variation of Svyk/Muy*Fz with kappa
RVY6 = 1.49012e-08             #Variation of Svyk/Muy*Fz with atan(kappa)
PPY1 = 0.0 #Pressure effect on cornering stiffness magnitude
PPY2 = 0.0 #Pressure effect on location of cornering stiffness peak
PPY3 = 0.0 #Linear pressure effect on lateral friction
PPY4 = 0 #Quadratic pressure effect on lateral friction
PPY5 = 0.0 #Influence of inflation pressure on camber stiffness
PTY1 = 0.0                  #Peak value of relaxation length SigAlp0/R0
PTY2 = 0.0                  #Value of Fz/Fznom where SigAlp0 is extreme


# Vehicle parameters
FZ0        = 2670
g          = 9.81
T = 0.002
la = 1.25
lb = 2.019-la  #2.715-la
h_cg = 0.596;
m = 1.269e+03
Izz = 1.620e+03
Fz = m*g # 12449
Fzf = Fz*lb/(la+lb) # 5332
Fzr = Fz-Fzf

#Fx and Fy are given in tire ref frame
#Fy_out is the projection of forces acting on vehicle in lateral direction
#front tire

dfz_f = (Fzf-FZ0)/FZ0 # 1

kappa = 0

SHx_f = (PHX1 + PHX2*dfz_f)
SVx_f = Fzf*(PVX1 + PVX2*dfz_f)
kappa_x_f = kappa + SHx_f
Cx_f = PCX1
Mux_f = (PDX1 + PDX2*dfz_f)
Dx_f = Mux_f*Fzf
Ex_f = 0.0

Kx_f = Fzf*(PKX1 + PKX2*dfz_f)*exp(PKX3*dfz_f)
Bx_f = Kx_f / (Cx_f*Dx_f)

Cy_f = PCY1 #1.4376
SVy_f = Fzf*(PVY1+PVY2*dfz_f)
SHy_f = PHY1+PHY2*dfz_f

#rear tire

dfz_r = (Fzr-FZ0)/FZ0

SHx_r = (PHX1 + PHX2*dfz_r)
SVx_r = Fzr*(PVX1 + PVX2*dfz_r)
kappa_x_r = kappa + SHx_r
Cx_r = PCX1
Mux_r = (PDX1 + PDX2*dfz_r)
Dx_r = Mux_r*Fzr
Ex_r = 0.0

Kx_r = Fzr*(PKX1 + PKX2*dfz_r)*exp(PKX3*dfz_r)
Bx_r = Kx_r / (Cx_r*Dx_r)



Cy_r = PCY1 #1.4376
SVy_r = Fzr*(PVY1+PVY2*dfz_r)
SHy_r = PHY1+PHY2*dfz_r

# tire parameters
KZX     = 289.5
KZYR    = 427.
KZYF    = 277.2
FzF0    = 2670.
FzR0    = 2670.
PC1     = PCY1;
PD1     = PDY1 - PDY2;
PD2     = PDY2/FZ0;
PE1     = PEY1 - PEY2;
PE2     = PEY2/FZ0;
PE3     = PEY3;
PK1     = PKY1*FZ0;
PK2     = 1/(PKY2*FZ0);
PH1     = PHY1 - PHY2;
PH2     = PHY2/FZ0;
PV1     = PVY1 - PVY2;
PV2     = PVY2/FZ0;

# vehicle Limits
ux = 3.
x_min    = 0.
x_max    = 400.
y_min    = 0.
y_max    = 400.
sa_min   = -0.485
sa_max   = 0.485
psi_min  = -2*pi
psi_max  = 2*pi
u_min    = 0.01   #5.
u_max    = 30.
sr_min   = -0.523
sr_max   = 0.523
jx_min   = -2.1*3
jx_max   = 1.9*3

Caf =  -3.69554	# cornering stiffness--front axle (N/rad)
Car =  -2.76883	# cornering stiffness-- rear axle (N/rad)
Fy_min = -75000000
Fy_max = 75000000

# constrained initial states
x0_     = 200.
y0_     = 0.
psi0_   = pi/2
v0_     = 0.
u0_     = 5.
sa0_    = 0.
sr0_    = 0.
ax0_    = 0.
jx0_    = 0.
r0_     = 0.

# leave these parameters here
Fz_min  = 200.
Fz_off  = 100.
a_t     = Fz_min + 3*Fz_off  # soft tire force constraint constants
b_t     = Fz_off
EP      = 0.01

dPKY = 0.


#neural net params
nn_offset1 =(0.);
nn_offset2 =(-0.99963);
nn_offset3 =(-0.59998);
nn_offset4 =(2.);
nn_offset5=(-0.55996);
nn_offset6 =(43615.);
nn_offset7 =(0.30014);
nn_offset8 =(651.23);
nn_offset9 =(6.0024);
nn_offset10=(0.010002);

nn_gain1 =(3.4272256060363e-05);
nn_gain2 =(1.00018203313003);
nn_gain3 =(1.66684724178453);
nn_gain4 =(0.25002187691423);
nn_gain5 =(1.78587373872667);
nn_gain6 =(9.8208432667071e-07);
nn_gain7 =(2.22281497288166);
nn_gain8 =(9.97617191338488e-05);
nn_gain9 =(0.0625066413306414);
nn_gain10 =(142.897970848814);

nn_min =(-1.);


#layer 1
b1_1 =(-0.062530550601044732062)
b1_2 =(3.914954425956560069)
b1_3 =(-10.32581771393949488)
b1_4 =(-1.4525192174571270876)
b1_5 =(0.30075085592122907663)
b1_6 =(1.2069766357692486292)
b1_7 =(4.0994857414410272867)
b1_8 =(0.48191466871590693533)
b1_9 =(0.36792310162283653474)
b1_10 =(-0.081176515331998322367)
b1_11 =(-1.0816701857474808612)
b1_12 =(-1.8285311399483006323)


IW1_1=(-0.044283263483106800884)
IW1_2=(0.050138327816753994193)
IW1_3=(1.9821583393283936925)
IW1_4=(-0.0027760193279775919294)
IW1_5=(-0.0088680021149649505829)
IW1_6=(-0.056596044155137484322)
IW1_7=(0.10474622714659730105)
IW1_8=(0.0048123257231001449752)
IW1_9=(0.032288245068123375137)
IW1_10=(-0.015738218205967786228)
IW2_1=(0.95245810824910526193)
IW2_2=(-0.04945982995140659616)
IW2_3=(-0.052217149637909382465)
IW2_4=(0.011993196212686433808)
IW2_5=(-0.020270456047017982454)
IW2_6=(1.458398574986951024)
IW2_7=(0.039005092440100010143)
IW2_8=(-0.321983616952055407)
IW2_9=(0.21066593131713701181)
IW2_10=(-0.027168145572185681963)
IW3_1=(-9.3995590841826128781)
IW3_2=(-0.061859354597240769069)
IW3_3=(0.02492727958819383402)
IW3_4=(0.0011377187301789179995)
IW3_5=(-0.0064497343750726217684)
IW3_6=(0.091170306275734913637)
IW3_7=(-0.17856165668885123909)
IW3_8=(-0.082630795955787886276)
IW3_9=(0.096527809472945924618)
IW3_10=(0.013992228264395922474)
IW4_1=(1.9145991280123089151)
IW4_2=(-0.31496273421765269562)
IW4_3=(-0.22198359691736890831)
IW4_4=(0.04700292977333401373)
IW4_5=(-0.068682207222792751589)
IW4_6=(0.16023962669606542364)
IW4_7=(-0.69965020738574090764)
IW4_8=(-0.45178944041136281928)
IW4_9=(0.40546754211502572529)
IW4_10=(-0.22675532217154359405)
IW5_1=(1.5554500654365432943)
IW5_2=(-3.2369616217474623809)
IW5_3=(1.6023019341594015863)
IW5_4=(0.0088752585603212516552)
IW5_5=(-0.048002226403515452224)
IW5_6=(-0.091509461763438729176)
IW5_7=(0.13748780162602361465)
IW5_8=(0.023576917323600968257)
IW5_9=(0.077134463104111788967)
IW5_10=(-0.019933362750869123431)
IW6_1=(0.98130143341241293786)
IW6_2=(-0.15050346927231730843)
IW6_3=(0.12640265233010605783)
IW6_4=(0.0033374238644615118743)
IW6_5=(0.0023439462946805889217)
IW6_6=(0.017755988070462112166)
IW6_7=(-0.043754234134112043875)
IW6_8=(-0.39515523499890320425)
IW6_9=(-0.43134148802612498619)
IW6_10=(0.01350638423412435507)
IW7_1=(-0.24727744778250823621)
IW7_2=(0.63933479044859731211)
IW7_3=(1.0173689466559334704)
IW7_4=(0.85826541837571168614)
IW7_5=(-0.12042191002152334567)
IW7_6=(-0.64891856437387640533)
IW7_7=(1.1442019064933426353)
IW7_8=(0.27103590137334410137)
IW7_9=(-0.44105449567316373782)
IW7_10=(-0.073630258598270603709)
IW8_1=(0.29608467528146903414)
IW8_2=(-0.48869844270058737656)
IW8_3=(-2.0853851313926230482)
IW8_4=(0.013245039463815899)
IW8_5=(0.0085981946272537957549)
IW8_6=(-0.014491139555036416237)
IW8_7=(-0.038689836634890110989)
IW8_8=(0.0019258122746274900731)
IW8_9=(-0.008549104143013179502)
IW8_10=(-0.031529066304618831584)
IW9_1=(-0.75355227837955440773)
IW9_2=(2.8243015426330493334)
IW9_3=(1.137107066226790586)
IW9_4=(-0.045227398618875001846)
IW9_5=(-0.01416643083881011754)
IW9_6=(0.096331724051735234671)
IW9_7=(-0.10588837451822115387)
IW9_8=(-0.012957671852650098562)
IW9_9=(-0.073449158006079026673)
IW9_10=(0.050598523715760200525)
IW10_1=(-0.090730701363303731255)
IW10_2=(-0.069066750951632685518)
IW10_3=(0.041714834392282844344)
IW10_4=(-0.011612759520064169089)
IW10_5=(0.0021678085467007825356)
IW10_6=(0.016992740533526787955)
IW10_7=(-0.10219828559109424282)
IW10_8=(-0.071652508767658773525)
IW10_9=(-0.18007367765989576447)
IW10_10=(0.025124753190916428169)
IW11_1=(-0.86319328751283219692)
IW11_2=(0.12820011051551991055)
IW11_3=(0.04187198968232552776)
IW11_4=(0.0019955538786033842277)
IW11_5=(-0.0058228894825549069869)
IW11_6=(-0.01186407891916757551)
IW11_7=(0.04289162486118076878)
IW11_8=(0.39509284412105105666)
IW11_9=(0.45115218399546935801)
IW11_10=(-0.01891127682275754035)
IW12_1=(-1.3873121001133730257)
IW12_2=(0.21215734676382522195)
IW12_3=(-0.098356043698532971686)
IW12_4=(0.049887183653939341788)
IW12_5=(-0.004641701601403638508)
IW12_6=(-0.10087140615764871032)
IW12_7=(0.38122143172867312133)
IW12_8=(-0.19652928114550319294)
IW12_9=(0.011044055994931648024)
IW12_10=(-0.073964651456691621334)



#Layer 2
b2_1 =(-1.4454501896446616538);
b2_2 =(-2.9393515874264788401);

LW21_1_1 =(1.4201526051355788383);
LW21_1_2 =(-1.9347976884114586049);
LW21_1_3 =(-3.6060792682514604124);
LW21_1_4 =(1.9070081261507769721);
LW21_1_5 =(-0.24812366668741875353);
LW21_1_6 =(0.73710948478173010656);
LW21_1_7 =(-1.4466157323879049734);
LW21_1_8 =(1.2468434371911760739);
LW21_1_9 =(0.73039134130930083444);
LW21_1_10 =(-3.0852626419226987231);
LW21_1_11 =(0.10824405133853498562);
LW21_1_12 =(-0.82212281332270542578);

LW21_2_1 =(1.1934906820125210647);
LW21_2_2 =(-2.026259724217573055);
LW21_2_3 =(-3.6898003890817587802);
LW21_2_4 =(2.0317610654370850121);
LW21_2_5 =(-0.61076936881372267951);
LW21_2_6 =(-0.055103039991048451129);
LW21_2_7 =(0.29783020147744726502);
LW21_2_8 =(1.2541836028406652126);
LW21_2_9 =(0.31377324910296411353);
LW21_2_10 =(-3.3759606208112606929);
LW21_2_11 =(-0.74831502256960602537);
LW21_2_12 =(-0.85258041633020853478);



#layer 3
b3 =(-0.4916460610542531251);

LW32_1=(-2.4751721435203850596);
LW32_2=(2.0533908446056807762);


ymin =(-1.);
gain =(7.17139159778246e-05);
yoffset =(-12968.26);



terraintype_Id = 5;

if terraintype_Id == 1
    # wet sand
    terrain_Kphi = 4021e3
    terrain_cohesion = 26.39e3
    terrain_phi = 23.5 # in degrees
    terrain_Kt = 4.02e-3

elseif terraintype_Id == 2
    # dry sand
    terrain_Kphi = 1.96e7
    terrain_cohesion = 7.98e3
    terrain_phi = 23.3
    terrain_Kt = 1.83e-3

elseif terraintype_Id == 3
    # clay
    terrain_Kphi = 705400
    terrain_cohesion = 4140
    terrain_phi = 13.178
    terrain_Kt = 0.01

elseif terraintype_Id == 4
    # grass
    terrain_Kphi = 1180000
    terrain_cohesion = 24000
    terrain_phi = 42
    terrain_Kt = 0.046

elseif terraintype_Id == 5
    # sandy loam
    terrain_Kphi = 1515000
    terrain_cohesion = 1700
    terrain_phi = 28.99
    terrain_Kt = 0.025

elseif terraintype_Id == 6
    # sand in T Gou's work
    terrain_Kphi = 1528600
    terrain_cohesion = 200
    terrain_phi = rad2deg(0.4712)
    terrain_Kt = 0.024

elseif terraintype_Id == 7
    # up land sandy loam
    terrain_Kphi = 1418e3
    terrain_cohesion = 3.3e3
    terrain_phi = rad2deg(0.588)
    terrain_Kt = 0.093

elseif terraintype_Id == 8
    # loose air dry sand
    terrain_Kphi = 754.13e3
    terrain_cohesion = 0
    terrain_phi = rad2deg(0.478)
    terrain_Kt = 0.005

elseif terraintype_Id == 9
    # medium soil
    terrain_Kphi = 2083e3
    terrain_cohesion = 8.62e3
    terrain_phi = rad2deg(0.393)
    terrain_Kt = 0.025

elseif terraintype_Id == 10
    # clayed soil
    terrain_Kphi = 499.7e3
    terrain_cohesion = 7.58e3
    terrain_phi = rad2deg(0.244)
    terrain_Kt = 0.0254
end
