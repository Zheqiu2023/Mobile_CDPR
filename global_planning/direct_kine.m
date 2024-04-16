%% 任意假定一个末端平台位姿，逆运动学求出绳长，然后将绳长作为已知数代入函数中求解正运动学，将求出的位姿与假定位姿相比较，验证结果是否正确
% 求解出来的位姿与假定位姿一般相差较大，原因在于末端平台有6自由度，只有4根绳驱动（欠驱动），无法求解；加上静力平衡方程后能求解，但求得的位
% 姿需满足静力平衡，而假定位姿下静力平衡不一定能满足，所以二者有差异。所以逆运动学不能只是利用几何关系求解绳长，应当引入力平衡方程
function [real_pose, real_cf] = direct_kine(z, l, d, plan_result_last, param_cdpr)
%% 求解正运动学
% for 4 cables with 6 DOF, cable force and EE position must be calculated together
x0=[plan_result_last(13:18); plan_result_last(5:8)];
% boundary
lb=[-10;-10;0;-1;-1;-pi;  0;0;0;0];
ub=[10;10;1;1;1;pi;    inf;inf;inf;inf];
% options
options = optimset('Display','final','TolFun',1e-21,'TolX',1e-21,'MaxIter',5000,'MaxFunEvals',5000*9);

% 更新锚点座坐标
param_cdpr.bp_coor(3, :) = z;

[result,resnorm,residual,exitflag,output] = lsqnonlin(@(x) direct_kine_lsqnonlin(x,l,d,param_cdpr),x0,lb,ub,options);

real_pose = result(1:6) % 实际位姿 
real_cf = result(7:10)  % 实际绳力


