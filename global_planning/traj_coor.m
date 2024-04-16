function [trj_type, param_trj] = traj_coor(ep_start,timestep)
% 初始位置
b0 = ep_start';

%% 直线轨迹
a1 = [2.1,1.2,0.6] + b0;

param_trj1 = {0,timestep,20,b0,a1};

param_trj = {param_trj1};
trj_type = {'line'};
%% 圆轨迹
% a1 = [2,3,0.6] + b0;
% a2 = [2,2.7,0.6] + b0;
% a3 = [2,2.3,0.6] + b0;
% 
% param_trj1={0,timestep,8,b0,a1};
% param_trj2={0,timestep,2,a1,a2};
% param_trj3={0,timestep,8,a2,a1,a2};
% param_trj4={0,timestep,2,a2,a3};
% param_trj5={0,timestep,12,a3,a1,a3};
% param_trj6={0,timestep,4,a3,a1};
% 
% param_trj={param_trj1,param_trj2,param_trj3,param_trj4,param_trj5,param_trj6};
% trj_type={'line','line','circle','line','circle','line'};
%% HIT轨迹
% a1 = [1,1.5,0.1] + b0;
% a2 = [1,1.5,0.7] + b0;
% a3 = [1,1.5,0.4] + b0;
% a4 = [1,1.9,0.4] + b0;
% a5 = [1,1.9,0.7] + b0;
% a6 = [1,1.9,0.1] + b0;
% a7 = [1,2.3,0.1] + b0;
% a8 = [1,2.3,0.7] + b0;
% a9 = [1,2.3,0.1] + b0;
% a10 = [1,2.7,0.1] + b0;
% a11 = [1,2.7,0.7] + b0;
% a12 = [1,2.5,0.7] + b0;
% a13 = [1,2.9,0.7] + b0;
% 
% param_trj1 = {0,timestep,5,b0,a1};
% param_trj2 = {0,timestep,6,a1,a2};
% param_trj3 = {0,timestep,3,a2,a3};
% param_trj4 = {0,timestep,4,a3,a4};
% param_trj5 = {0,timestep,3,a4,a5};
% param_trj6 = {0,timestep,6,a5,a6};
% param_trj7 = {0,timestep,4,a6,a7};
% param_trj8 = {0,timestep,6,a7,a8};
% param_trj9 = {0,timestep,6,a8,a9};
% param_trj10 = {0,timestep,4,a9,a10};
% param_trj11 = {0,timestep,6,a10,a11};
% param_trj12 = {0,timestep,2,a11,a12};
% param_trj13 = {0,timestep,4,a12,a13};
% 
% param_trj = {param_trj1,param_trj2,param_trj3,param_trj4,param_trj5,param_trj6,param_trj7,...
%             param_trj8,param_trj9,param_trj10,param_trj11,param_trj12,param_trj13};
% trj_type = {'line','line','line','line','line','line','line','line','line','line','line','line','line'};



