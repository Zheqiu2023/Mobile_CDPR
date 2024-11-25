clear, close all, fclose all;
delete("ep.xlsx"); delete("ep.csv"); delete("ep_target_pose.csv"); delete("car.csv");
param_cdpr = param_mobile_cdpr;

%% 处理末端轨迹
load('抬高末端/抬高末端.mat');
% load('绕行/绕行.mat');
num = size(ep_traj, 2);
L = 1.114;
W = 1.17;
for i = 1:num
    car_traj(1, i) = car_traj(1, i) + L*cos(car_traj(3, i))/2;
    car_traj(2, i) = car_traj(2, i) + L*sin(car_traj(3, i))/2;
    car_traj(1:6, i) = [car_traj(1, i);car_traj(2, i);0;0;0;car_traj(3, i)];
    ep_traj(:, i) = ep_traj(:, i) - car_traj(:, i) + [param_cdpr.ep_o_g(1:3);0;0;0];
end

writematrix(ep_traj(1, :),'ep.xlsx','WriteMode','append');
writematrix(ep_traj(2, :),'ep.xlsx','WriteMode','append');
writematrix(ep_traj(3, :),'ep.xlsx','WriteMode','append');
fid1 = fopen('ep_target_pose.csv', 'a', 'n', 'UTF-8');
fid2 = fopen('ep.csv', 'a', 'n', 'UTF-8');    % 创建一个csv文件,追加数据到文件末尾

%% 求关节转角和实际位姿
t_vec = (1:1:num) * param_cdpr.timestep;
plan_result = zeros(18,num);  plan_result_last = [param_cdpr.bp_z_init;zeros(4,1);param_cdpr.cl_init;zeros(6,1)]; % 保存上一时刻的规划结果
for i = 1:num
    pose_traj = ep_traj(:, i);
    [real_z, ~, real_cl] = inverse_kine(pose_traj, plan_result_last, param_cdpr);
    [real_pose, real_cf] = direct_kine(real_z, real_cl, plan_result_last, param_cdpr);
    
    plan_result(:,i) = [real_z;real_cf;real_cl;real_pose]; 
    plan_result_last = [real_z;real_cf;real_cl;real_pose];    % 用来保证相邻轨迹点的求解结果不跳变
    
    % 正运动学求解的期望位姿，写入文件中，与实际位姿对比
    fprintf(fid1, '%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n', real_pose);% 一行6个数据，用逗号分隔，每行结束后加上\n换行
    % 将结果写入.csv文件，用来控制电机运行
    fprintf(fid2, '%.5f,', real_z-param_cdpr.bp_z_init);
    fprintf(fid2, '%.5f,%.5f,%.5f,%.5f\n', real_cl-param_cdpr.cl_init);% 一行8个数据，用逗号分隔，每行结束后加上\n换行 
end

% 将求解结果写入文件，以便查看
fclose(fid1);
fclose(fid2);
writematrix(plan_result(13:18,:),'ep.xlsx','WriteMode','append');
writematrix(plan_result(1:4,:),'ep.xlsx','Sheet',2,'WriteMode','append');
writematrix(plan_result(9:12,:),'ep.xlsx','Sheet',3,'WriteMode','append');
writematrix(plan_result(5:8,:),'ep.xlsx','Sheet',4,'WriteMode','append');

plot_trajectory_gif(t_vec, plan_result(13:18,:), param_cdpr.bp_coor);

%% calculate and save alpha&v for each wheelset
alpha = zeros(num, 4);
v = zeros(num, 4);

Vref = car_v(1, :)'; % 后轴中心速度
ALPHAref = car_v(2, :)'; % 后轴中心转角
R = L./tan(ALPHAref); % 转弯半径
Wref = Vref./R;% 转向角速度 

% Ackerman steering 
alpha(:, 1) = atan(L./(R-W.*sign(ALPHAref)/2));
alpha(:, 2) = atan(L./(R+W.*sign(ALPHAref)/2));
v(:, 1) = Wref.*L./sin(alpha(:, 1));
v(:, 2) = Wref.*L./sin(alpha(:, 2));
v(:, 3) = Wref.*(R-sign(ALPHAref).*W/2);
v(:, 4) = Wref.*(R+sign(ALPHAref).*W/2);
writematrix([alpha v],'car.csv','WriteMode','append');
