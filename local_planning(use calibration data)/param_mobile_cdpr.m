function param_cdpr = param_mobile_cdpr
%% 4根绳索的起始固定位置 单位m
% 动锚点座Z向位置最小值和最大值
bp_z_min = [-0.7702;-0.76854;-0.76961;-0.76788];   % 标定值
bp_z_max = [-0.26577;-0.26428;-0.26532;-0.26343];

bp_x = [1.12817;1.13552;0.0113;0.01507]; % 标定初始值
bp_y = [0.44575;-0.63909;0.4381;-0.65492];  

init_h = [0.15;0.15;0.15;0.15];   % 锚点座初始高度
cl_init = [1.3213;1.3355;1.3157;1.3373];    % 锚点座初始绳长
% init_h = [0.1;0.05;0.05;0.1];   % 锚点座初始高度
% cl_init = [1.3372;1.3704;1.3494;1.3541];    % 锚点座初始绳长

bp_1 = [bp_x(1); bp_y(1); 0; 1];
bp_2 = [bp_x(2); bp_y(2); 0; 1];
bp_3 = [bp_x(3); bp_y(3); 0; 1];
bp_4 = [bp_x(4); bp_y(4); 0; 1];
bp_coor = [bp_1 bp_2 bp_3 bp_4];

param_cdpr.init_h = init_h;
param_cdpr.bp_z_max = bp_z_max;
param_cdpr.bp_z_min = bp_z_min;
param_cdpr.cl_init = cl_init;
param_cdpr.bp_coor = bp_coor;
%% 末端平台的绳索固定位置 单位m
ep_x = 40e-3;
ep_y = 40e-3;
ep_z = 30e-3;

ep_1 = [ep_x;  ep_y;  ep_z; 1];
ep_2 = [ep_x; -ep_y;  ep_z; 1];
ep_3 = [-ep_x; ep_y;  ep_z; 1];
ep_4 = [-ep_x; -ep_y; ep_z; 1];
ep_o = [0;0;0;1];   % 末端平台重心在局部坐标系下的坐标
ep_o_g = [0.56903;-0.08651;-1.31197;1];   % 末端平台重心在全局坐标系下的坐标

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
param_cdpr.max_h_step = 0.000167;
param_cdpr.max_cl_step = 0.003142;
