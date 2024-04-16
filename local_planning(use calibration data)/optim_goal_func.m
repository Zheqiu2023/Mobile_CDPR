function optim_goal = optim_goal_func(x, plan_result_last, param_cdpr)
% 优化目标
optim_h = sumsqr((x(1:4)-param_cdpr.init_h)/0.504);  
optim_cf = sumsqr(x(5:8)/500);
optim_cl = sumsqr((x(9:12)-param_cdpr.cl_init)/1);

optim_h_dot = sumsqr((plan_result_last(1:4)-x(1:4))/param_cdpr.max_h_step);  % Z坐标变化速度
optim_cl_dot = sumsqr((plan_result_last(9:12)-x(9:12))/param_cdpr.max_cl_step);  % 绳长变化速度

optim_goal = 0.5*optim_h_dot+0.0001*optim_cf+0.4999*optim_cl_dot;   % 优化绳力的加权系数应尽量小
% optim_goal = 0.3009*optim_h+0.0001*optim_cf+0.099*optim_cl+0.5*optim_h_dot+0.1*optim_cl_dot;% 可行
% optim_goal = 0.8*optim_cf+0.1*optim_h+0.1*optim_cl;% 竖直轨迹可行