function [pos, vel, accel, eular, t, N] = trajPlan2(pose0, posef, v_max, a_max, t0, dt)
%说明：trajPlan2(pose0, posef, v_max, a_max, t0, dt)
%五次多项式插值直线轨迹规划函数
%input：目标姿态(坐标、欧拉角)以及起动时间、最大速度上限、最大加速度上限、时间步
%output：插值后的位置pos（3XN），速度vel（3XN），加速度accel（3XN），ZYX欧拉角eular（3XN）,时间序列t（1XN）,点数N
%功能：对位置坐标和ZYX欧拉角直接进行五次多项式插值规划

%% 确保位姿数组为列向量
if size(pose0,2) ~=1
    pose0 = pose0';
elseif size(posef,2) ~=1
    posef = posef';
end

P0 = pose0(1:3);
Pf = posef(1:3);
eular0 = pose0(4:6);
eularf = posef(4:6);

%% 以最大速度和加速度条件求运动时间（0<=tau<=1）
tau_vMax = 0.5; %五次多项式归一化速度最大值时tau的值
tau_aMax = 0.5 - sqrt(3)/6; %五次多项式归一化加速度最大值时tau的值
dlamda5_dtau_Max = 30*tau_vMax^2 - 60*tau_vMax^3 + 30*tau_vMax^4; %五次多项式归一化速度最大值
ddlamda5_dtau_Max = 60*tau_aMax - 180*tau_aMax^2 + 120*tau_aMax^3; %五次多项式归一化加速度最大值
distance = norm(Pf - P0);
t0_f_bound1 = dlamda5_dtau_Max * distance / v_max; % 最大速度决定的最小运动时间
t0_f_bound2 = sqrt(ddlamda5_dtau_Max * distance / a_max); % 最大加速度决定的最小运动时间
% 取两个下限中的最大值
if t0_f_bound1>=t0_f_bound2
    t0_f = t0_f_bound1;
else
    t0_f = t0_f_bound2;
end
t0_f = round(t0_f, 2); %四舍五入到2位小数
t0_f = ceil(t0_f * 10) / 10; %向上舍入到1位小数

%% 基本参数与时间序列
tf = t0 + t0_f;
t = t0:dt:tf;
tau = (t-t0) / t0_f; %归一化时间
N = size(tau,2);

lamda5=10*tau.^3-15*tau.^4+6*tau.^5; %五次多项式位置规划(归一化)
dlamda5_dtau=30*tau.^2-60*tau.^3+30*tau.^4; %五次多项式速度规划(归一化)
ddlamda5_dtau=60*tau-180*tau.^2+120*tau.^3; %五次多项式加速度规划(归一化)

%% 规划
pos=P0+(Pf - P0).*lamda5;
vel=dlamda5_dtau.*(Pf - P0) / t0_f;
accel=ddlamda5_dtau.*(Pf - P0) / (t0_f * t0_f);
eular = eular0+(eularf - eular0).*lamda5;

end