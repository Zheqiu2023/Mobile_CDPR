clear, close all

%% Define the following model parameters.
% * |L| (cdpr length)
% * |W| (cdpr width)
% * |Lwheel| (wheel diameter)
% * |Wwheel| (wheel width)
params = struct('L',1.114,...
    'W',1.17,...
    'Lwheel',0.235,...
    'Wwheel',0.05);

% Time settings and variables
T = 50; % Trajectory final time
dt = 0.05; % time step duration
K = T/dt + 1; % number of time steps
k_hor = 50; % prediction horizon length
t_vec = 0:dt:T;

ep_start_pose = [-4;0;0;0;0;0];  % 末端平台起点位姿
ep_end_pose = [4;0;0;0;0;0];  % 末端平台终点位姿
car_start_pose = [ep_start_pose(1)-params.L/2;ep_start_pose(2);ep_start_pose(6)];  % 车身起点位姿
car_end_pose = [ep_end_pose(1)-params.L/2*cos(ep_end_pose(6));ep_end_pose(2)-params.L/2*sin(ep_end_pose(6));ep_end_pose(6)];

start_state = [ep_start_pose;  zeros(6,1);  car_start_pose;  zeros(3,1)];   % 起始状态量
start_input = zeros(8,1);

%% 获取参考轨迹
ep_waypts = [ep_start_pose [-2;1;0;0;0;pi/4] [2;1;0;0;0;-pi/4] ep_end_pose];
ep_ref_traj = minimum_snap_simple(ep_waypts, T);
ep_ref_traj(:,end+1:end+k_hor) = repmat(ep_ref_traj(:, end),1,k_hor);

%% 求解实际轨迹
state_result = start_state;  % 状态量
input_result = start_input; % 输入量

% for i = 1:K
% %     ref_in = CDPR_Near_Distance_cal(start_state,start_input,ep_ref_traj(1:6, :),ep_ref_traj(7:12, :),ep_ref_traj(13:18, :),k_hor);
% %     [Input, State] = Next_Point_Planning(start_state, start_input, ref_in, k_hor, dt, params);
%     [Input, State] = Next_Point_Planning(start_state, start_input, ep_ref_traj(:, i:i+k_hor), k_hor, dt, params);
%     start_input = Input(:,1);
%     start_state = State(:,2);
%     state_result = [state_result start_state];
%     input_result = [input_result start_input];
% end

error_sum=zeros(3,1);
error_p=zeros(3,1);
for i = 1:6000  % 最多执行6000个循环，跳出循环说明轨迹没有收敛
    if norm(start_state(1:3) - ep_end_pose(1:3)) > 0.1
        [Input, State] = Next_Point_Planning(start_state, start_input, ep_ref_traj(:, i:i+k_hor), k_hor, dt, params);
        start_input = Input(:,1);
        start_state = State(:,2);
        state_result = [state_result start_state];
        input_result = [input_result start_input];
    else
        % 给定轨迹pid将目标拉回
        [CDPR1_State_c,CDPR1_Input_c,error_sum,error_p] = PIDcontrol(CDPR2_Pose_target,CDPR1_State_p,error_sum,error_p,dt);
        CDPR1_State_p=[CDPR1_State_c;0;0;0];
        CDPR1_Input_p=CDPR1_Input_c;
    end
end

ep_traj = state_result(1:6, :);
ep_v = state_result(7:12, :);
car_traj = state_result(13:15, :);
car_v = state_result(16:17, :);
% 轨迹动图
figure(1)
plot(ep_start_pose(1), ep_start_pose(2), 'go');hold on;
plot(ep_end_pose(1), ep_end_pose(2), 'go');hold on;
% plot(car_start_pose(1), car_start_pose(2), 'go');hold on;
% plot(car_end_pose(1), car_end_pose(2), 'go');
CdprPlot(ep_traj, car_traj, params, t_vec, 0);hold on
plot(ep_ref_traj(1, :), ep_ref_traj(2, :));
% 末端平台轨迹结果
figure(2)
subplot(2,6,1),plot(t_vec,state_result(1,1:length(t_vec)));title('x position');
subplot(2,6,2),plot(t_vec,state_result(2,1:length(t_vec)));title('y position');
subplot(2,6,3),plot(t_vec,state_result(3,1:length(t_vec)));title('z position');
subplot(2,6,4),plot(t_vec,state_result(4,1:length(t_vec)));title('alpha angle');
subplot(2,6,5),plot(t_vec,state_result(5,1:length(t_vec)));title('beta angle');
subplot(2,6,6),plot(t_vec,state_result(6,1:length(t_vec)));title('gamma angle');
subplot(2,6,7),plot(t_vec,state_result(7,1:length(t_vec)));title('x velocity');
subplot(2,6,8),plot(t_vec,state_result(8,1:length(t_vec)));title('y velocity');
subplot(2,6,9),plot(t_vec,state_result(9,1:length(t_vec)));title('z velocity');
subplot(2,6,10),plot(t_vec,state_result(10,1:length(t_vec)));title('alpha velocity');
subplot(2,6,11),plot(t_vec,state_result(11,1:length(t_vec)));title('beta velocity');
subplot(2,6,12),plot(t_vec,state_result(12,1:length(t_vec)));title('gamma velocity');
% 车身轨迹结果
figure(3)
subplot(2,3,1),plot(t_vec,state_result(13,1:length(t_vec)));title('x position');
subplot(2,3,2),plot(t_vec,state_result(14,1:length(t_vec)));title('y position');
subplot(2,3,3),plot(t_vec,state_result(15,1:length(t_vec)));title('theta angle');
subplot(2,3,4),plot(t_vec,state_result(16,1:length(t_vec)));title('v');
subplot(2,3,5),plot(t_vec,state_result(17,1:length(t_vec)));title('α');



