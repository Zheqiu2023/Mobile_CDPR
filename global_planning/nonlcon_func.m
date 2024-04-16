function [c,ceq] = nonlcon_func(x,pose_ee_des,plan_result_last,param_cdpr)
% 合外力
force_ee=[0;0;-param_cdpr.M_ee*param_cdpr.g];
% 合外力矩
moment_ee=[0;0;0];
%% 约束方程
% x = [Z(1) Z(2) Z(3) Z(4) F(1) F(2) F(3) F(4) L(1) L(2) L(3) L(4) D(1) D(2) D(3)]
param_cdpr.bp_coor(3, :) = x(1:4); % 将动锚点座初始Z向位置作为未知数，代入方程求解    
pose_cdpr_des = [x(13) x(14) x(15)];
[jaco_trans,ideal_cl] = calc_jaco(param_cdpr,pose_ee_des,pose_cdpr_des);        

%% 不等式约束
% 约束z坐标变化速度
% c(1) = (x(1)-plan_result_last(1))^2-(1e-2)^2;
% c(2) = (x(2)-plan_result_last(2))^2-(1e-2)^2;
% c(3) = (x(3)-plan_result_last(3))^2-(1e-2)^2;
% c(4) = (x(4)-plan_result_last(4))^2-(1e-2)^2;
% 约束绳长变化速度
% c(5) = (x(9)-plan_result_last(9))^2-(1e-2)^2;
% c(6) = (x(10)-plan_result_last(10))^2-(1e-2)^2;
% c(7) = (x(11)-plan_result_last(11))^2-(1e-2)^2;
% c(8) = (x(12)-plan_result_last(12))^2-(1e-2)^2;
% 约束绳力变化速度
% c(1) = (x(5)-plan_result_last(5))^2-10^2;
% c(2) = (x(6)-plan_result_last(6))^2-10^2;
% c(3) = (x(7)-plan_result_last(7))^2-10^2;
% c(4) = (x(8)-plan_result_last(8))^2-10^2;
% 约束cdpr转角变化速度
c(5) = (x(15)-plan_result_last(21))^2-(1e-4)^2;
%% 10个等式约束方程求解15个未知数
% 绳长约束方程
ceq(1:4)=x(9:12)-ideal_cl;
% 力平衡方程 J'*T+F=0
equ_j=jaco_trans*x(5:8)+[force_ee;moment_ee];
ceq(5:10)=equ_j;
% ceq=ceq*1e6;