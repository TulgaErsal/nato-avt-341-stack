"""
@file mpc_parameters.jl

@brief Models for the model predictive control planner.

@date 10/12/2023

@author Tulga Ersal (tersal@umich.edu)
        University of Michigan
        Department of Mechanical Engineering
"""

function ThreeDOF_rigid(n)
    # lateral tire force
    FzF = :($FzF0 - $KZX * (-(v[j] * r[j]) + ax[j]))
    FzR = :($FzR0 + $KZX * (-(v[j] * r[j]) + ax[j]))
  
    X1F = :($p1 / $mu * (atan((v[j] + $la * r[j]) / ($EP + ux[j])) - sa[j]))
    X1R = :($p1 / $mu * (atan((v[j] - $lb * r[j]) / ($EP + ux[j]))))

    FYF = :(2 * $mu * $FzF * sin($p2 * atan($X1F - $p3 * ($X1F - atan($X1F)))))
    FYR = :(2 * $mu * $FzR * sin($p2 * atan($X1R - $p3 * ($X1R - atan($X1R)))))

    # vehicle dynamics
    dx = Array{Expr}(undef,8)
    dx[1] = :(ux[j]*cos(psi[j]) - (v[j] + $la*r[j])*sin(psi[j]))   # X position of front axle
    dx[2] = :(ux[j]*sin(psi[j]) + (v[j] + $la*r[j])*cos(psi[j]))   # Y position of front axle
    dx[3] = :(($FYF + $FYR)/$m - r[j]*ux[j])                       # Lateral speed
    dx[4] = :(($la*$FYF-$lb*$FYR)/$Izz)                            # Yaw rate
    dx[5] = :(r[j])                                                # Yaw angle
    dx[6] = :(sr[j])                                               # Front wheel steering angle
    dx[7] = :(ax[j])                                               # Longitudinal speed
    dx[8] = :(jx[j])                                               # Longitudinal acceleration
    return dx
end

function ThreeDOF_linear(n)
  # lateral tire load
  FYF=:(2 * $Caf * $Fzf * (atan((v[j] + $la*r[j])/(ux[j]+$EP))-sa[j]))
  FYR=:(2 * $Car * $Fzr * atan((v[j] - $lb*r[j])/(ux[j]+$EP)))

  dx = Array{Expr}(undef,8)
  dx[1] = :(ux[j]*cos(psi[j]) - (v[j] + $la*r[j])*sin(psi[j]))   # X position of front axle
  dx[2] = :(ux[j]*sin(psi[j]) + (v[j] + $la*r[j])*cos(psi[j]))   # Y position of front axle
  dx[3] = :(($FYF + $FYR)/$m - r[j]*ux[j])                       # Lateral speed
  dx[4] = :(($la*$FYF-$lb*$FYR)/$Izz)                            # Yaw rate
  dx[5] = :(r[j])                                                # Yaw angle
  dx[6] = :(sr[j])                                               # Front wheel steering angle
  dx[7] = :(ax[j])                                               # Longitudinal speed
  dx[8] = :(jx[j])                                               # Longitudinal acceleration
  return dx
end


function ThreeDOF_deformable(n, n_f, n_r, f_kappa, r_kappa)

  # lateral tire load

  #----------- rear tire neural net------------------------------------
  alpha_r =:(atan( ((v[j]-$lb*r[j])/(ux[j]+$RBY3))));
  W_r=:(0.5*($m*$g*$la - $m*r[j]*v[j]*$h_cg)/($la+$lb));
  wheel_speed_rx =:(ux[j]);
  wheel_speed_ry =:(v[j]-$lb*r[j]);
  speed =:(sqrt($wheel_speed_rx^2. + $wheel_speed_ry^2.));
  da_r=:(0.0);

  nn_in1 =:($W_r);
  nn_in2=:($r_kappa);
  nn_in3 =:($alpha_r);
  nn_in4=:($speed);
  nn_in5=:($da_r);
  nn_in6 =:($terrain_Kphi);
  nn_in7=:($n_r);
  nn_in8 =:($terrain_cohesion);
  nn_in9=:($terrain_phi);
  nn_in10=:($terrain_Kt);

  xp1_1 =:(((($nn_in1-$nn_offset1)*$nn_gain1)+$nn_min))
  xp1_2 =:(((($nn_in2-$nn_offset2)*$nn_gain2)+$nn_min))
  xp1_3 =:(((($nn_in3-$nn_offset3)*$nn_gain3)+$nn_min))
  xp1_4 =:(((($nn_in4-$nn_offset4)*$nn_gain4)+$nn_min))
  xp1_5 =:(((($nn_in5-$nn_offset5)*$nn_gain5)+$nn_min))
  xp1_6 =:(((($nn_in6-$nn_offset6)*$nn_gain6)+$nn_min))
  xp1_7 =:(((($nn_in7-$nn_offset7)*$nn_gain7)+$nn_min))
  xp1_8 =:(((($nn_in8-$nn_offset8)*$nn_gain8)+$nn_min))
  xp1_9 =:(((($nn_in9-$nn_offset9)*$nn_gain9)+$nn_min))
  xp1_10 =:(((($nn_in10-$nn_offset10)*$nn_gain10)+$nn_min))


  #apply layer 1
  layer_in1=:($b1_1+$IW1_1*$xp1_1+$IW1_2*$xp1_2+$IW1_3*$xp1_3+$IW1_4*$xp1_4+$IW1_5*$xp1_5+$IW1_6*$xp1_6+$IW1_7*$xp1_7+$IW1_8*$xp1_8+$IW1_9*$xp1_9+$IW1_10*$xp1_10);
  layer_in2=:($b1_2+$IW2_1*$xp1_1+$IW2_2*$xp1_2+$IW2_3*$xp1_3+$IW2_4*$xp1_4+$IW2_5*$xp1_5+$IW2_6*$xp1_6+$IW2_7*$xp1_7+$IW2_8*$xp1_8+$IW2_9*$xp1_9+$IW2_10*$xp1_10);
  layer_in3=:($b1_3+$IW3_1*$xp1_1+$IW3_2*$xp1_2+$IW3_3*$xp1_3+$IW3_4*$xp1_4+$IW3_5*$xp1_5+$IW3_6*$xp1_6+$IW3_7*$xp1_7+$IW3_8*$xp1_8+$IW3_9*$xp1_9+$IW3_10*$xp1_10);
  layer_in4=:($b1_4+$IW4_1*$xp1_1+$IW4_2*$xp1_2+$IW4_3*$xp1_3+$IW4_4*$xp1_4+$IW4_5*$xp1_5+$IW4_6*$xp1_6+$IW4_7*$xp1_7+$IW4_8*$xp1_8+$IW4_9*$xp1_9+$IW4_10*$xp1_10);
  layer_in5=:($b1_5+$IW5_1*$xp1_1+$IW5_2*$xp1_2+$IW5_3*$xp1_3+$IW5_4*$xp1_4+$IW5_5*$xp1_5+$IW5_6*$xp1_6+$IW5_7*$xp1_7+$IW5_8*$xp1_8+$IW5_9*$xp1_9+$IW5_10*$xp1_10);
  layer_in6=:($b1_6+$IW6_1*$xp1_1+$IW6_2*$xp1_2+$IW6_3*$xp1_3+$IW6_4*$xp1_4+$IW6_5*$xp1_5+$IW6_6*$xp1_6+$IW6_7*$xp1_7+$IW6_8*$xp1_8+$IW6_9*$xp1_9+$IW6_10*$xp1_10);
  layer_in7=:($b1_7+$IW7_1*$xp1_1+$IW7_2*$xp1_2+$IW7_3*$xp1_3+$IW7_4*$xp1_4+$IW7_5*$xp1_5+$IW7_6*$xp1_6+$IW7_7*$xp1_7+$IW7_8*$xp1_8+$IW7_9*$xp1_9+$IW7_10*$xp1_10);
  layer_in8=:($b1_8+$IW8_1*$xp1_1+$IW8_2*$xp1_2+$IW8_3*$xp1_3+$IW8_4*$xp1_4+$IW8_5*$xp1_5+$IW8_6*$xp1_6+$IW8_7*$xp1_7+$IW8_8*$xp1_8+$IW8_9*$xp1_9+$IW8_10*$xp1_10);
  layer_in9=:($b1_9+$IW9_1*$xp1_1+$IW9_2*$xp1_2+$IW9_3*$xp1_3+$IW9_4*$xp1_4+$IW9_5*$xp1_5+$IW9_6*$xp1_6+$IW9_7*$xp1_7+$IW9_8*$xp1_8+$IW9_9*$xp1_9+$IW9_10*$xp1_10);
  layer_in10=:($b1_10+$IW10_1*$xp1_1+$IW10_2*$xp1_2+$IW10_3*$xp1_3+$IW10_4*$xp1_4+$IW10_5*$xp1_5+$IW10_6*$xp1_6+$IW10_7*$xp1_7+$IW10_8*$xp1_8+$IW10_9*$xp1_9+$IW10_10*$xp1_10);
  layer_in11=:($b1_11+$IW11_1*$xp1_1+$IW11_2*$xp1_2+$IW11_3*$xp1_3+$IW11_4*$xp1_4+$IW11_5*$xp1_5+$IW11_6*$xp1_6+$IW11_7*$xp1_7+$IW11_8*$xp1_8+$IW11_9*$xp1_9+$IW11_10*$xp1_10);
  layer_in12=:($b1_12+$IW12_1*$xp1_1+$IW12_2*$xp1_2+$IW12_3*$xp1_3+$IW12_4*$xp1_4+$IW12_5*$xp1_5+$IW12_6*$xp1_6+$IW12_7*$xp1_7+$IW12_8*$xp1_8+$IW12_9*$xp1_9+$IW12_10*$xp1_10);



  #apply layer 1
  layer_output1 =:(2.0/(1.0 + exp(-2.0*$layer_in1)) - 1.0);
  layer_output2 =:(2.0/(1.0 + exp(-2.0*$layer_in2)) - 1.0);
  layer_output3 =:(2.0/(1.0 + exp(-2.0*$layer_in3)) - 1.0);
  layer_output4 =:(2.0/(1.0 + exp(-2.0*$layer_in4)) - 1.0);
  layer_output5 =:(2.0/(1.0 + exp(-2.0*$layer_in5)) - 1.0);
  layer_output6 =:(2.0/(1.0 + exp(-2.0*$layer_in6)) - 1.0);
  layer_output7 =:(2.0/(1.0 + exp(-2.0*$layer_in7)) - 1.0);
  layer_output8 =:(2.0/(1.0 + exp(-2.0*$layer_in8)) - 1.0);
  layer_output9 =:(2.0/(1.0 + exp(-2.0*$layer_in9)) - 1.0);
  layer_output10 =:(2.0/(1.0 + exp(-2.0*$layer_in10)) - 1.0);
  layer_output11 =:(2.0/(1.0 + exp(-2.0*$layer_in11)) - 1.0);
  layer_output12 =:(2.0/(1.0 + exp(-2.0*$layer_in12)) - 1.0);


  #  #apply layer 2
  layer_input1=:($b2_1+$LW21_1_1*$layer_output1+$LW21_1_2*$layer_output2+$LW21_1_3*$layer_output3+$LW21_1_4*$layer_output4+$LW21_1_5*$layer_output5+$LW21_1_6*$layer_output6+$LW21_1_7*$layer_output7+$LW21_1_8*$layer_output8+$LW21_1_9*$layer_output9+$LW21_1_10*$layer_output10+$LW21_1_11*$layer_output11+$LW21_1_12*$layer_output12);
  layer_input2=:($b2_2+$LW21_2_1*$layer_output1+$LW21_2_2*$layer_output2+$LW21_2_3*$layer_output3+$LW21_2_4*$layer_output4+$LW21_2_5*$layer_output5+$LW21_2_6*$layer_output6+$LW21_2_7*$layer_output7+$LW21_2_8*$layer_output8+$LW21_2_9*$layer_output9+$LW21_2_10*$layer_output10+$LW21_2_11*$layer_output11+$LW21_2_12*$layer_output12);


  layer_output2_1 =:(2.0/(1.0 + exp(-2.0*$layer_input1)) - 1.0);
  layer_output2_2 =:(2.0/(1.0 + exp(-2.0*$layer_input2)) - 1.0);




  nn_out=:($b3+$LW32_1*$layer_output2_1+$LW32_2*$layer_output2_2);

  FYR =:(2.0*((($nn_out-$ymin)/$gain)+$yoffset));
  #--------------------------------------------------------------------
  #----------- front tire neural net-----------------------------------
  da_f =:(sr[j]*$T*$T);
  alpha_f =:(atan( ((v[j]+$la*r[j])/(ux[j]+$RBY3))) - sa[j]);
  W_f=:(0.5*($m*$g*$lb + $m*r[j]*v[j]*$h_cg)/($la+$lb));
  wheel_speed_fx =:(ux[j]);
  wheel_speed_fy =:(v[j]+$la*r[j]);

  speed_f =:(sqrt($wheel_speed_fx^2. + $wheel_speed_fy^2.));

  nn_in1f =:($W_f);
  nn_in2f=:($f_kappa);
  nn_in3f =:($alpha_f);
  nn_in4f=:($speed_f);
  nn_in5f=:($da_f);
  nn_in6f =:($terrain_Kphi);
  nn_in7f=:($n_f);
  nn_in8f =:($terrain_cohesion);
  nn_in9f=:($terrain_phi);
  nn_in10f=:($terrain_Kt);

  xp1_1f =:(((($nn_in1f-$nn_offset1)*$nn_gain1)+$nn_min))
  xp1_2f =:(((($nn_in2f-$nn_offset2)*$nn_gain2)+$nn_min))
  xp1_3f =:(((($nn_in3f-$nn_offset3)*$nn_gain3)+$nn_min))
  xp1_4f =:(((($nn_in4f-$nn_offset4)*$nn_gain4)+$nn_min))
  xp1_5f =:(((($nn_in5f-$nn_offset5)*$nn_gain5)+$nn_min))
  xp1_6f =:(((($nn_in6f-$nn_offset6)*$nn_gain6)+$nn_min))
  xp1_7f =:(((($nn_in7f-$nn_offset7)*$nn_gain7)+$nn_min))
  xp1_8f =:(((($nn_in8f-$nn_offset8)*$nn_gain8)+$nn_min))
  xp1_9f =:(((($nn_in9f-$nn_offset9)*$nn_gain9)+$nn_min))
  xp1_10f =:(((($nn_in10f-$nn_offset10)*$nn_gain10)+$nn_min))

  #apply layer 1
  layer_in1f=:($b1_1+$IW1_1*$xp1_1f+$IW1_2*$xp1_2f+$IW1_3*$xp1_3f+$IW1_4*$xp1_4f+$IW1_5*$xp1_5f+$IW1_6*$xp1_6f+$IW1_7*$xp1_7f+$IW1_8*$xp1_8f+$IW1_9*$xp1_9f+$IW1_10*$xp1_10f);
  layer_in2f=:($b1_2+$IW2_1*$xp1_1f+$IW2_2*$xp1_2f+$IW2_3*$xp1_3f+$IW2_4*$xp1_4f+$IW2_5*$xp1_5f+$IW2_6*$xp1_6f+$IW2_7*$xp1_7f+$IW2_8*$xp1_8f+$IW2_9*$xp1_9f+$IW2_10*$xp1_10f);
  layer_in3f=:($b1_3+$IW3_1*$xp1_1f+$IW3_2*$xp1_2f+$IW3_3*$xp1_3f+$IW3_4*$xp1_4f+$IW3_5*$xp1_5f+$IW3_6*$xp1_6f+$IW3_7*$xp1_7f+$IW3_8*$xp1_8f+$IW3_9*$xp1_9f+$IW3_10*$xp1_10f);
  layer_in4f=:($b1_4+$IW4_1*$xp1_1f+$IW4_2*$xp1_2f+$IW4_3*$xp1_3f+$IW4_4*$xp1_4f+$IW4_5*$xp1_5f+$IW4_6*$xp1_6f+$IW4_7*$xp1_7f+$IW4_8*$xp1_8f+$IW4_9*$xp1_9f+$IW4_10*$xp1_10f);
  layer_in5f=:($b1_5+$IW5_1*$xp1_1f+$IW5_2*$xp1_2f+$IW5_3*$xp1_3f+$IW5_4*$xp1_4f+$IW5_5*$xp1_5f+$IW5_6*$xp1_6f+$IW5_7*$xp1_7f+$IW5_8*$xp1_8f+$IW5_9*$xp1_9f+$IW5_10*$xp1_10f);
  layer_in6f=:($b1_6+$IW6_1*$xp1_1f+$IW6_2*$xp1_2f+$IW6_3*$xp1_3f+$IW6_4*$xp1_4f+$IW6_5*$xp1_5f+$IW6_6*$xp1_6f+$IW6_7*$xp1_7f+$IW6_8*$xp1_8f+$IW6_9*$xp1_9f+$IW6_10*$xp1_10f);
  layer_in7f=:($b1_7+$IW7_1*$xp1_1f+$IW7_2*$xp1_2f+$IW7_3*$xp1_3f+$IW7_4*$xp1_4f+$IW7_5*$xp1_5f+$IW7_6*$xp1_6f+$IW7_7*$xp1_7f+$IW7_8*$xp1_8f+$IW7_9*$xp1_9f+$IW7_10*$xp1_10f);
  layer_in8f=:($b1_8+$IW8_1*$xp1_1f+$IW8_2*$xp1_2f+$IW8_3*$xp1_3f+$IW8_4*$xp1_4f+$IW8_5*$xp1_5f+$IW8_6*$xp1_6f+$IW8_7*$xp1_7f+$IW8_8*$xp1_8f+$IW8_9*$xp1_9f+$IW8_10*$xp1_10f);
  layer_in9f=:($b1_9+$IW9_1*$xp1_1f+$IW9_2*$xp1_2f+$IW9_3*$xp1_3f+$IW9_4*$xp1_4f+$IW9_5*$xp1_5f+$IW9_6*$xp1_6f+$IW9_7*$xp1_7f+$IW9_8*$xp1_8f+$IW9_9*$xp1_9f+$IW9_10*$xp1_10f);
  layer_in10f=:($b1_10+$IW10_1*$xp1_1f+$IW10_2*$xp1_2f+$IW10_3*$xp1_3f+$IW10_4*$xp1_4f+$IW10_5*$xp1_5f+$IW10_6*$xp1_6f+$IW10_7*$xp1_7f+$IW10_8*$xp1_8f+$IW10_9*$xp1_9f+$IW10_10*$xp1_10f);
  layer_in11f=:($b1_11+$IW11_1*$xp1_1f+$IW11_2*$xp1_2f+$IW11_3*$xp1_3f+$IW11_4*$xp1_4f+$IW11_5*$xp1_5f+$IW11_6*$xp1_6f+$IW11_7*$xp1_7f+$IW11_8*$xp1_8f+$IW11_9*$xp1_9f+$IW11_10*$xp1_10f);
  layer_in12f=:($b1_12+$IW12_1*$xp1_1f+$IW12_2*$xp1_2f+$IW12_3*$xp1_3f+$IW12_4*$xp1_4f+$IW12_5*$xp1_5f+$IW12_6*$xp1_6f+$IW12_7*$xp1_7f+$IW12_8*$xp1_8f+$IW12_9*$xp1_9f+$IW12_10*$xp1_10f);

  #apply layer 1
  layer_output1f =:(2.0/(1.0 + exp(-2.0*$layer_in1f)) - 1.0);
  layer_output2f =:(2.0/(1.0 + exp(-2.0*$layer_in2f)) - 1.0);
  layer_output3f =:(2.0/(1.0 + exp(-2.0*$layer_in3f)) - 1.0);
  layer_output4f =:(2.0/(1.0 + exp(-2.0*$layer_in4f)) - 1.0);
  layer_output5f =:(2.0/(1.0 + exp(-2.0*$layer_in5f)) - 1.0);
  layer_output6f =:(2.0/(1.0 + exp(-2.0*$layer_in6f)) - 1.0);
  layer_output7f =:(2.0/(1.0 + exp(-2.0*$layer_in7f)) - 1.0);
  layer_output8f =:(2.0/(1.0 + exp(-2.0*$layer_in8f)) - 1.0);
  layer_output9f =:(2.0/(1.0 + exp(-2.0*$layer_in9f)) - 1.0);
  layer_output10f =:(2.0/(1.0 + exp(-2.0*$layer_in10f)) - 1.0);
  layer_output11f =:(2.0/(1.0 + exp(-2.0*$layer_in11f)) - 1.0);
  layer_output12f =:(2.0/(1.0 + exp(-2.0*$layer_in12f)) - 1.0);

  #  #apply layer 2
  layer_input1f=:($b2_1+$LW21_1_1*$layer_output1f+$LW21_1_2*$layer_output2f+$LW21_1_3*$layer_output3f+$LW21_1_4*$layer_output4f+$LW21_1_5*$layer_output5f+$LW21_1_6*$layer_output6f+$LW21_1_7*$layer_output7f+$LW21_1_8*$layer_output8f+$LW21_1_9*$layer_output9f+$LW21_1_10*$layer_output10f+$LW21_1_11*$layer_output11f+$LW21_1_12*$layer_output12f);
  layer_input2f=:($b2_2+$LW21_2_1*$layer_output1f+$LW21_2_2*$layer_output2f+$LW21_2_3*$layer_output3f+$LW21_2_4*$layer_output4f+$LW21_2_5*$layer_output5f+$LW21_2_6*$layer_output6f+$LW21_2_7*$layer_output7f+$LW21_2_8*$layer_output8f+$LW21_2_9*$layer_output9f+$LW21_2_10*$layer_output10f+$LW21_2_11*$layer_output11f+$LW21_2_12*$layer_output12f);

  layer_output2_1f =:(2.0/(1.0 + exp(-2.0*$layer_input1f)) - 1.0);
  layer_output2_2f =:(2.0/(1.0 + exp(-2.0*$layer_input2f)) - 1.0);




  nn_outf=:($b3+$LW32_1*$layer_output2_1f+$LW32_2*$layer_output2_2f);

  FYF =:(2.0*((($nn_outf-$ymin)/$gain)+$yoffset));

  #----------- bicycle model------------------------------------

  dx = Array{Expr}(undef,8)
  dx[1] = :(ux[j]*cos(psi[j]) - (v[j] + $la*r[j])*sin(psi[j]))   # X position of front axle
  dx[2] = :(ux[j]*sin(psi[j]) + (v[j] + $la*r[j])*cos(psi[j]))   # Y position of front axle
  dx[3] = :(($FYF + $FYR)/$m - r[j]*ux[j])                       # Lateral speed
  dx[4] = :(($la*$FYF-$lb*$FYR)/$Izz)                            # Yaw rate
  dx[5] = :(r[j])                                                # Yaw angle
  dx[6] = :(sr[j])                                               # Steering angle
  dx[7] = :(ax[j])                                               # Longitudinal speed
  dx[8] = :(jx[j])                                               # Longitudinal acceleration

  return dx
end
