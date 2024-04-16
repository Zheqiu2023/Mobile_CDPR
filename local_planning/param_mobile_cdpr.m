function param_cdpr = param_mobile_cdpr
%% 4根绳索的起始固定位置 单位m
% 动锚点座Z向位置最小值和最大值
% bp_z_min = [0.577;0.577;0.577;0.577];   % 理论值
% bp_z_max = [1.09;1.09;1.09;1.09;1.09];
bp_z_min = [0.58;0.58;0.58;0.58];   % 估计值
bp_z_max = bp_z_min + 0.5;

% bp_x = [0.557;0.557;-0.557;-0.557]; % 理论值
% bp_y = [0.545;-0.545;0.545;-0.545]; 
bp_x = [0.5544;0.5544;-0.5544;-0.5544]; % 估计值
bp_y = [0.5446;-0.5446;0.5446;-0.5446]; 

bp_z_init = bp_z_min + 0.15;
cl_init = [1.3423;1.3423;1.3423;1.3423];
% bp_z_init = bp_z_min + [0.175;0.05;0.05;0.175];
% cl_init = [1.3332;1.3756;1.3756;1.3332];

bp_1 = [bp_x(1); bp_y(1); bp_z_init(1); 1];
bp_2 = [bp_x(2); bp_y(2); bp_z_init(2); 1];
bp_3 = [bp_x(3); bp_y(3); bp_z_init(3); 1];
bp_4 = [bp_x(4); bp_y(4); bp_z_init(4); 1];
bp_coor = [bp_1 bp_2 bp_3 bp_4];

param_cdpr.bp_z_init = bp_z_init;
param_cdpr.bp_z_max = bp_z_max;
param_cdpr.bp_z_min = bp_z_min;
param_cdpr.cl_init = cl_init;
param_cdpr.bp_coor = bp_coor;
%% 末端平台的绳索固定位置 单位m
ep_x = 25e-3;
ep_y = 25e-3;
ep_z = 30e-3;

ep_1 = [ep_x;  ep_y;  ep_z; 1];
ep_2 = [ep_x; -ep_y;  ep_z; 1];
ep_3 = [-ep_x; ep_y;  ep_z; 1];
ep_4 = [-ep_x; -ep_y; ep_z; 1];
ep_o = [0;0;0;1];
ep_o_g = [0;0;0.028;1];

ep_coor = [ep_1 ep_2 ep_3 ep_4];
param_cdpr.ep_coor = ep_coor;
param_cdpr.ep_o = ep_o;
param_cdpr.ep_o_g = ep_o_g;
%% 末端平台质量 
M_ee = 1.0;   % kg
g = 9.81;
param_cdpr.M_ee = M_ee;
param_cdpr.g = g;
%% 绞盘
param_cdpr.dia_winch = 30e-3*ones(4,1); % 绞盘直径 单位m
%% 滑轮
dia_cable = 0.5;  % 绳索直径 单位mm
dia_pulley = 9.4;   % 滑轮直径 单位mm
param_cdpr.pulley_radius = (dia_pulley/2+dia_cable)*1e-3;   
param_cdpr.rotation_radius = 17e-3;     % 滑轮组旋转半径 单位m
%% 轨迹步长
param_cdpr.timestep = 0.01;
param_cdpr.max_z_step = 0.000167;
param_cdpr.max_cl_step = 0.003142;
