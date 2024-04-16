function optim_goal = optim_goal_func(x, plan_result_last, param_cdpr)
%% 优化目标  归一化+非负化处理
optim_z = sumsqr(((x(1:4)-param_cdpr.bp_z_init)/0.5));  % 优化Z坐标
optim_cf = sumsqr((x(5:8)/500)); % 优化绳力
optim_cl = sumsqr(((x(9:12)-param_cdpr.cl_init)/2.5));  % 优化绳长
optim_xy_cdpr = sumsqr((x(13:14)/20));  % 优化cdpr在xy方向的移动距离
optim_angle_cdpr = (x(15)/(2*pi))^2;  % 优化cdpr绕z轴转角

optim_z_dot = sumsqr((plan_result_last(1:4)-x(1:4))/param_cdpr.max_z_step);  % 优化Z坐标变化速度
optim_cf_dot = sumsqr((plan_result_last(5:8)-x(5:8))/100); % 优化绳力变化速度
optim_cl_dot = sumsqr((plan_result_last(9:12)-x(9:12))/param_cdpr.max_cl_step); % 优化绳长变化速度
optim_xy_cdpr_dot = sumsqr((plan_result_last(19:20)-x(13:14))); % 优化cdpr移动距离变化速度
optim_angle_cdpr_dot =  ((plan_result_last(21)-x(15))/pi)^2; % 优化cdpr转角变化速度

%% 多目标优化——线性加权法：权因子总和为1
optim_goal = 0.5*optim_cf_dot+0.1*optim_z_dot+0.1*optim_cl_dot+0.25*optim_angle_cdpr_dot+0.05*optim_xy_cdpr_dot;
% optim_goal = 0.59*optim_cf_dot+0.01*optim_z_dot+0.01*optim_cl_dot+0.34*optim_angle_cdpr_dot+0.05*optim_xy_cdpr_dot;
% optim_goal = 0.2*optim_z_dot+0.8*optim_cl_dot;