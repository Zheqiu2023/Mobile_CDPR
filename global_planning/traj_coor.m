function [trj_type, param_trj] = traj_coor(ep_start,timestep)
% 初始位置
b0 = ep_start';

%% 直线轨迹
% a1 = [2.1,1.2,0.6] + b0;
% 
% param_trj1 = {0,timestep,20,b0,a1};
% 
% param_trj = {param_trj1};
% trj_type = {'line'};
%% 圆轨迹
a1 = [1,2,0.6] + b0;
a2 = [1,1,0.6] + b0;
a3 = [1,3,0.6] + b0;

param_trj1={0,timestep,20,b0,a1};
param_trj2={0,timestep,10,a1,a2};
param_trj3={0,timestep,50,a2,a1,a2};

param_trj={param_trj1,param_trj2,param_trj3};
trj_type={'line','line','circle'};



