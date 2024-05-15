% 规划圆轨迹
function [t_vec,plan_result] = plan_traj_circle(param_traj,plan_result_last,param_cdpr)
t_start = param_traj{1}; t_step = param_traj{2}; t_end = param_traj{3};
p_start = param_traj{4}; p_center = param_traj{5}; p_end = param_traj{6};
total_time = t_end-t_start;     % 总运行时间
t_vec = 0:t_step:total_time;
t_num = length(t_vec);

start_x=p_start(1); start_y=p_start(2); start_z=p_start(3);
c_x=p_center(1); c_y=p_center(2); c_z=p_center(3);% 圆心
end_x=p_end(1); end_y=p_end(2); end_z=p_end(3);
%% 求圆轨迹上所有轨迹点的xyz坐标
A_start = angle((start_x-c_x) + (start_y-c_y)*1i);  % -pi~pi
A_end = angle((end_x-c_x)+(end_y-c_y)*1i);
if(A_end<=A_start)
    A_end=A_end+2*pi;
end
% 角度规划
A_vec = traj_generator.QuinticInterpolation(A_start, 0, 0, A_end, 0, 0, t_vec);

% 求xyz坐标
radius=norm([start_x,start_y,start_z]-[c_x,c_y,c_z]);
traj_x=radius*cos(A_vec)+c_x; 
traj_y=radius*sin(A_vec)+c_y; 
traj_z=0*ones(1,t_num)+c_z;
% 将规划结果写入文件，以和求解结果作对比
writematrix(traj_x,'test.xlsx','WriteMode','append');   % 保存末端平台期望位置，期望姿态设为[0;0;0]
writematrix(traj_y,'test.xlsx','WriteMode','append');
writematrix(traj_z,'test.xlsx','WriteMode','append');
fid1 = fopen('ee_target_pose.csv', 'a', 'n', 'UTF-8');
fid2 = fopen('test.csv', 'a', 'n', 'UTF-8');    % 创建一个csv文件,追加数据到文件末尾
%% 求实际位姿
traj_Ax = zeros(1,t_num); traj_Ay = zeros(1,t_num); traj_Az = zeros(1,t_num);
plan_result = zeros(21,t_num);
for i=1:t_num
    pose_traj = [traj_x(i);traj_y(i);traj_z(i);traj_Ax(i);traj_Ay(i);traj_Az(i)];
    [real_z, ~, real_cl, real_pose_cdpr] = inverse_kine(pose_traj, plan_result_last, param_cdpr);
    [real_pose, real_cf] = direct_kine(real_z, real_cl, real_pose_cdpr, plan_result_last, param_cdpr);

    steer_angle = atan2(real_pose_cdpr(2)-plan_result_last(20), real_pose_cdpr(1)-plan_result_last(19)); % 车轮转向角度，rad
    roll_angle = sqrt(sumsqr(real_pose_cdpr(1:2)-plan_result_last(19:20)))/param_cdpr.wheel_radius; % 车轮滚转角度，rad

    plan_result(:,i) = [real_z;real_cf;real_cl;real_pose;real_pose_cdpr]; 
    plan_result_last = [real_z;real_cf;real_cl;real_pose;real_pose_cdpr];% 用来保证相邻轨迹点的求解结果不跳变

    % 正运动学求解的期望位姿，写入文件中，与实际位姿对比
    fprintf(fid1, '%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n', real_pose);    % 一行6个数据，用逗号分隔，每行结束后加上\n换行
    % 将结果写入.csv文件，用来控制电机运行
    fprintf(fid2, '%.5f,', real_z-param_cdpr.bp_z_init);    % 一行10个数据，用逗号分隔，每行结束后加上\n换行 
    fprintf(fid2, '%.5f,%.5f,%.5f,%.5f,', real_cl-param_cdpr.cl_init);
    fprintf(fid2, '%.5f,%.5f\n', steer_angle, roll_angle);
end

% 将求解结果写入文件，以便查看
fclose(fid1); 
fclose(fid2);
writematrix(plan_result(13:18,:),'test.xlsx','WriteMode','append');   % 保存末端平台位姿
writematrix(plan_result(19:21,:),'test.xlsx','Sheet',2,'WriteMode','append');   % 保存cdpr位姿
writematrix(plan_result(1:4,:),'test.xlsx','Sheet',3,'WriteMode','append');   % 保存Z坐标
writematrix(plan_result(9:12,:),'test.xlsx','Sheet',4,'WriteMode','append');   % 保存绳长
writematrix(plan_result(5:8,:),'test.xlsx','Sheet',5,'WriteMode','append');   % 保存绳力