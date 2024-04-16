function [c,ceq] = nonlcon_func(x,pose_des,plan_result_last,param)
% 合外力
force_ee=[0;0;-param.M_ee*param.g];
% 合外力矩
moment_ee=[0;0;0];
%% 约束方程
% 更新锚点座坐标
[X, Y, Z] = fit_archor_coor(x(1:4));
param.bp_coor(1, :) = X;
param.bp_coor(2, :) = Y;
param.bp_coor(3, :) = Z;

% 计算力雅可比矩阵和绳长
[jaco_trans,ideal_cl] = calc_jaco(param,pose_des);        
% 力平衡方程 J'*T+F=0
equ_j = jaco_trans*x(5:8) + [force_ee;moment_ee];
% 10个方程求解12个未知数
c(1:4) = abs(x(1:4) - plan_result_last(1:4)) - param.max_h_step; % 锚点座步长约束
ceq(1:4) = x(9:12) - ideal_cl;
ceq(5:10) = equ_j;