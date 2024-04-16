%% 给定末端位姿，求出绳索拉力、绳长和动锚点座Z向位置——10个方程，12个未知数
function [bp_z, ideal_cf, ideal_cl, pose_cdpr] = inverse_kine(pose_ee_des, plan_result_last, param_cdpr)
%% 求解逆运动学
options = optimset('Display','final','Algorithm','sqp','TolFun',1e-21,'TolX',1e-21,...
                   'TolCon',1e-15,'MaxIter',5000,'MaxFunEvals',5000*9);
% [Z(1) Z(2) Z(3) Z(4) F(1) F(2) F(3) F(4) L(1) L(2) L(3) L(4) D(1) D(2) D(3)]
x0 = [plan_result_last(1:12,:);plan_result_last(19:21,:)];% 初始参数给上一时刻的计算结果，以缩短计算时间
lb = [param_cdpr.bp_z_min;  0;0;0;0;    0.01;0.01;0.01;0.01;     -10;-10;-pi/4];% 下限
ub = [param_cdpr.bp_z_max;  inf;inf;inf;inf;   2.5;2.5;2.5;2.5;     10;10;pi/4];% 上限

[result,fval,exitflag,output]=fmincon(@(x) optim_goal_func(x,plan_result_last,param_cdpr),x0,[],[],[],[],lb,ub,...
                                      @(x) nonlcon_func(x,pose_ee_des,plan_result_last,param_cdpr),options);

bp_z = result(1:4)
ideal_cf = result(5:8)
ideal_cl = result(9:12)
pose_cdpr = result(13:15)   % x y angle_z

% 检验约束是否满足
% [c,ceq]=nonlcon_func(result,pose_ee_des,plan_result_last);
% if sum(ceq.^2)>1e-6
%     warning("约束不满足，优化失败")
% end
% 检验绳索张力是否过大
% if ~isempty(find(ideal_cf>1000,1))
%     warning("绳索张力过大")
% end
