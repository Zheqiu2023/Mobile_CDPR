clear,clc,close all
param_cdpr = param_mobile_cdpr;
%% 读取末端平台期望位置
ee_target_pose = readmatrix('data_2/ee_target_pose_updown.csv');
ee_target_pose = ee_target_pose';
%% 处理指令数据
send_data = readmatrix('data_2/send_local_traj11_07_16_08_26.csv');
point_cnt = size(send_data, 1);
% 初始时间
base_time = send_data(1, 1);
% 修改时间序列
cmd_t_vec = (send_data(:, 1) - base_time) / 1000;
% 修改位置数据
send_data(:, 2:end) = send_data(:, 2:end);   % m转换成mm

%% 处理电机反馈数据
recv_data = readmatrix('data_2/recv_traj11_07_16_08_18.csv');
for i = 1:4
    recv_leg_data{i} = recv_data(recv_data(:, 2) == i, :);
    % 修改时间序列
    recv_leg_data{i}(:, 1) = (recv_leg_data{i}(:, 1) - base_time) / 1000;
    % 修改位置数据
    recv_leg_data{i}(:, 3) = Cable_Convert_Pos(recv_leg_data{i}(:, 3));     % 绳索位置
    recv_leg_data{i}(:, 5) = Archor_Convert_Pos(recv_leg_data{i}(:, 5));    % 锚点座位置
    % 去除时间为负的数据
    recv_leg_data{i} = recv_leg_data{i}(recv_leg_data{i}(:, 1) >= 0, :);
end

% 修改位置数据
recv_leg_data{1}(:, 3) = -recv_leg_data{1}(:, 3);
recv_leg_data{4}(:, 3) = -recv_leg_data{4}(:, 3);

%% 计算误差
for i = 1:4
    for j = 1:point_cnt
        [~,ind] = min(abs(recv_leg_data{i}(:, 1) - cmd_t_vec(j)));
        cable_err(i, j) = recv_leg_data{i}(ind, 3) - send_data(j, 2*i);
        archor_err(i, j) = recv_leg_data{i}(ind, 5) - send_data(j, 2*i+1);
    end
    max_cable_err(i) = max(abs(cable_err(i, :))); 
    max_archor_err(i) = max(abs(archor_err(i, :)));
end

%% 绘图
% 绳索
figure(1);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cmd_t_vec, send_data(:, 2*i), 'r-', ...
         recv_leg_data{i}(:, 1), recv_leg_data{i}(:, 3), 'b--', 'LineWidth', 1.2);
    ylabel('位置(mm)');
    xlabel('时间(s)'); xlim([0 12]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cmd_t_vec, cable_err(i, :), '-.', 'LineWidth',1.2);
    ylabel('误差(mm)');
    legend('期望位置','实际位置','位置误差','FontSize',5,'EdgeColor','none');
    title(['绳索', num2str(i)],'FontSize',10);
end

% 锚点座
figure(2);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cmd_t_vec, send_data(:, 2*i+1), 'r-', ...
         recv_leg_data{i}(:, 1), recv_leg_data{i}(:, 5), 'b--', 'LineWidth', 1.2);
    ylabel('位置(mm)');
    xlabel('时间(s)'); xlim([0 12]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cmd_t_vec, archor_err(i, :), '-.', 'LineWidth',1.2);
    ylabel('误差(mm)');
    legend('期望位置','实际位置','位置误差','FontSize',5,'EdgeColor','none');
    title(['锚点座', num2str(i)],'FontSize',10);
end
%% 正运动学求解末端平台实际位置，与期望位置做对比
[num,ind] = min([size(recv_leg_data{1},1), size(recv_leg_data{2},1), size(recv_leg_data{3},1), size(recv_leg_data{4},1)]);
cur_timestamp = recv_leg_data{ind}(:,1);

% 末端平台实际位置、期望位置
ee_real_pose = zeros(6,num);
real_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.028;0;0;0];

for j = 1:num
    for i = 1:4
        [~,ind] = min(abs(recv_leg_data{i}(:,1) - cur_timestamp(j)));
        ccp(i) = recv_leg_data{i}(ind, 3) + param_cdpr.cl_init(i);
        acp(i) = recv_leg_data{i}(ind, 5) + param_cdpr.bp_z_init(i);
    end

    [res, ~] = direct_kine(acp(:), ccp(:), real_pose_last, param_cdpr)
    ee_real_pose(:,j) = res;
    real_pose_last(13:18) = res;
end

% target_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.028;0;0;0];
% for j = 1:point_cnt
%     for i = 1:4
%         ccp(i) = send_data(j, 2*i) + param_cdpr.cl_init(i);
%         acp(i) = send_data(j, 2*i+1) + param_cdpr.bp_z_init(i);
%     end
% 
%     [res, ~] = direct_kine(acp(:), ccp(:), target_pose_last, param_cdpr)
%     ee_target_pose(:,j) = res;
%     target_pose_last(13:18) = res;
% end

% 计算末端平台位置误差
ep_pose_err = zeros(6, point_cnt); max_ep_pose_err = zeros(6, 1); 
for i = 1:point_cnt
    [~,ind] = min(abs(cur_timestamp - cmd_t_vec(i)));
    ep_pose_err(:, i) = ee_target_pose(:, i) - ee_real_pose(:, ind);
end

for i = 1:6
    max_ep_pose_err(i) = max(abs(ep_pose_err(i, :)));
end

% 绘图
figure(3);
title('末端平台轨迹误差分析','FontSize',10);
for i = 1:3
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cmd_t_vec,ee_target_pose(i,:),'r-',...
         cur_timestamp,ee_real_pose(i,:),'b--','LineWidth',1.2);
    ylabel('位置(m)');
    xlabel('时间(s)');xlim([0 12]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cmd_t_vec,ep_pose_err(i,:),'-.','LineWidth',1.2);
    ylabel('误差(m)');
    legend('期望位置','实际位置','FontSize',5,'EdgeColor','none');
end
for i = 4:6
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cmd_t_vec,ee_target_pose(i,:)*180/pi,'r-',...
         cur_timestamp,ee_real_pose(i,:)*180/pi,'b--','LineWidth',1.2);
    ylabel('角度(°)');
    xlabel('时间(s)');xlim([0 12]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cmd_t_vec,ep_pose_err(i,:)*180/pi,'-.','LineWidth',1.2);
    ylabel('误差(°)');
    legend('期望位置','实际位置','FontSize',5,'EdgeColor','none');
end

%%
function m_pos = Cable_Convert_Pos(pos)
    RE35_REDUCTION_RATIO = 35;
    ENCODER_LINES_NUM = 2000;
    REEL_D = 30;
    m_pos = pos * pi * REEL_D / (1000 * RE35_REDUCTION_RATIO * ENCODER_LINES_NUM);   % qc转换成m
end

function m_pos = Archor_Convert_Pos(pos)
    LEAD = 5;
    RE35_REDUCTION_RATIO = 35;
    ENCODER_LINES_NUM = 2000;
    m_pos = pos * LEAD / (1000 * RE35_REDUCTION_RATIO * ENCODER_LINES_NUM);   % qc转换成m
end