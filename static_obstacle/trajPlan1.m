function [pos, vel, accel, eular, t, N] = trajPlan1(pose0, posef, t0, tf, dt)
%说明：trajPlan1(pose0, posef, t0, tf, dt)
%五次多项式插值直线轨迹规划函数
%input：目标姿态(坐标、欧拉角)以及起止时间、时间步
%output：插值后的位置pos（3XN），速度vel（3XN），加速度accel（3XN），ZYX欧拉角eular（3XN）,时间序列t（1XN）,点数N
%功能：对位置坐标和ZYX欧拉角直接进行五次多项式插值规划

%% 基本参数与时间序列
t = t0:dt:tf;
tau = (t-t0)/(tf-t0); %归一化时间
N = size(tau,2);

%确保位姿数组为列向量
if size(pose0,2) ~=1
    pose0 = pose0';
elseif size(posef,2) ~=1
    posef = posef';
end

P0 = pose0(1:3);
Pf = posef(1:3);
eular0 = pose0(4:6);
eularf = posef(4:6);

lamda5=10*tau.^3-15*tau.^4+6*tau.^5; %五次多项式位置规划(归一化)
dlamda5_dtau=30*tau.^2-60*tau.^3+30*tau.^4; %五次多项式速度规划(归一化)
ddlamda5_dtau=60*tau-180*tau.^2+120*tau.^3; %五次多项式加速度规划(归一化)

%% 规划
pos=P0+(Pf - P0).*lamda5
vel=dlamda5_dtau.*(Pf - P0) / (tf - t0);
accel=ddlamda5_dtau.*(Pf - P0) / ((tf - t0) * (tf - t0));
eular = eular0+(eularf - eular0).*lamda5;

end