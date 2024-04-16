function [c,ceq] = nonlcon_func(x,pose_des,plan_result_last, param_cdpr)
% 合外力
force_ee=[0;0;-param_cdpr.M_ee*param_cdpr.g];
% 合外力矩
moment_ee=[0;0;0];
%% 约束方程
% x = [Z(1) Z(2) Z(3) Z(4) F(1) F(2) F(3) F(4) L(1) L(2) L(3) L(4)]
param_cdpr.bp_coor(3, :) = x(1:4); % 将动锚点座初始Z向位置作为未知数，代入方程求解

[jaco_trans,ideal_cl] = calc_jaco(param_cdpr,pose_des);        
% force equilibrium equations J'*T+F=0
equ_j=jaco_trans*[x(5);x(6);x(7);x(8)]+[force_ee;moment_ee];
% 10个方程求解12个未知数
c(1:4) = abs(x(1:4) - plan_result_last(1:4)) - param_cdpr.max_z_step;
ceq(1:4) = x(9:12) - ideal_cl(1:4);
ceq(5:10) = equ_j(1:6);