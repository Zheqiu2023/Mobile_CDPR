%% 计算初始绳长
clear, clc
param_cdpr = param_mobile_cdpr;

%% 计算初始z坐标和绳长
[X, Y, Z] = fit_archor_coor(param_cdpr.init_h);
param_cdpr.bp_coor(1, :) = X;
param_cdpr.bp_coor(2, :) = Y;
param_cdpr.bp_coor(3, :) = Z;

%% 对于给定末端的执行器的位置 计算雅克比矩阵
% 末端执行器上绳索固定点的坐标
a1_e = param_cdpr.ep_coor(:,1);
a2_e = param_cdpr.ep_coor(:,2);
a3_e = param_cdpr.ep_coor(:,3);
a4_e = param_cdpr.ep_coor(:,4);
ao_e = param_cdpr.ep_o;

% 绳索起始点的坐标
b1_g = param_cdpr.bp_coor(:,1);
b2_g = param_cdpr.bp_coor(:,2);
b3_g = param_cdpr.bp_coor(:,3);
b4_g = param_cdpr.bp_coor(:,4);

% 末端执行器坐标系到全局坐标系的姿态变换矩阵 
Trans = transl(param_cdpr.ep_o_g(1:3));  % 末端平台处于初始位置 
% 全局坐标系下末端执行器上的点的坐标
a1_g = Trans*a1_e; 
a2_g = Trans*a2_e;
a3_g = Trans*a3_e;
a4_g = Trans*a4_e;
ao_g = Trans*ao_e;

ideal_length1 = calc_cable_length(a1_g(1:3),b1_g(1:3),param_cdpr.pulley_radius,param_cdpr.rotation_radius,param_cdpr.bp_z_max(1))
ideal_length2 = calc_cable_length(a2_g(1:3),b2_g(1:3),param_cdpr.pulley_radius,param_cdpr.rotation_radius,param_cdpr.bp_z_max(2))
ideal_length3 = calc_cable_length(a3_g(1:3),b3_g(1:3),param_cdpr.pulley_radius,param_cdpr.rotation_radius,param_cdpr.bp_z_max(3))
ideal_length4 = calc_cable_length(a4_g(1:3),b4_g(1:3),param_cdpr.pulley_radius,param_cdpr.rotation_radius,param_cdpr.bp_z_max(4))