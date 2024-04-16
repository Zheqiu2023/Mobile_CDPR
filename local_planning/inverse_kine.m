%% 给定末端位姿，求出绳索拉力、绳长和动锚点座Z向位置——10个方程，12个未知数
function [bp_z, ideal_cf, ideal_cl] = inverse_kine(pose_des, plan_result_last, param_cdpr)
%% 求解逆运动学
% 此设置求解较慢，可适当减小TolFun，TolX的阶次以提高运算速度
options = optimset('Display','final','Algorithm','sqp','TolFun',1e-21,'TolX',1e-21,'MaxIter',5000);
% [Z(1) Z(2) Z(3) Z(4) F(1) F(2) F(3) F(4) L(1) L(2) L(3) L(4)]
x0 = plan_result_last(1:12);
lb = [param_cdpr.bp_z_min;  0;0;0;0;    0.01;0.01;0.01;0.01];% 下限
ub = [param_cdpr.bp_z_max;  500;500;500;500;   2.5;2.5;2.5;2.5;];% 上限

[result,fval,exitflag,output] = fmincon(@(x) optim_goal_func(x,plan_result_last,param_cdpr),x0,[],[],[],[],lb,ub,...
                                        @(x) nonlcon_func(x,pose_des,plan_result_last,param_cdpr),options);
    
bp_z = result(1:4)
ideal_cf = result(5:8);
ideal_cl = result(9:12)

