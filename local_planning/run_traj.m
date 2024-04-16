clear, close all, fclose all;
delete("test.xlsx"); delete("test.csv"); delete("ee_target_pose.csv");

%% 按轨迹运行
param_cdpr = param_mobile_cdpr;
pose_plot = [];t_plot = [];z_plot = [];
plan_result_last = [param_cdpr.bp_z_init;zeros(4,1);param_cdpr.cl_init;zeros(6,1)]; % 保存上一时刻的规划结果

[trj_type, param_traj] = traj_coor(param_cdpr.ep_o_g(1:3), param_cdpr.timestep);    % 轨迹参数
for i = 1:length(param_traj)
    switch trj_type{i}
        case 'line'
            [t_vec,plan_result] = plan_traj_line(param_traj{i},plan_result_last,param_cdpr);
        case 'circle'
            [t_vec,plan_result] = plan_traj_circle(param_traj{i},plan_result_last,param_cdpr);
    end
    plan_result_last = plan_result(:,end);   % 用来保证两段轨迹连接点处的求解结果不跳变 
    plot_trajectory_gif(t_vec, plan_result(13:18,:), param_cdpr.bp_coor)

    pose_plot = [pose_plot,plan_result(13:18,:)];
    t_plot = [t_plot,t_vec]; z_plot = [z_plot,plan_result(1:4,:)];
end
% plot_robot_motion(t_plot,pose_plot,z_plot,param_cdpr);



