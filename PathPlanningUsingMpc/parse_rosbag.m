clear, close all
file = 'obs';
% bag = rosbag([file,'.bag']);
bag = rosbag('data/single_motor_obs_2_0.15.bag');
motor_num = 1;
%% ��ȡA1Ŀ���ٶ�
msgs(1) = select(bag,'Topic','/traj_vel_cmd0');
msgs(2) = select(bag,'Topic','/traj_vel_cmd1');
msgs(3) = select(bag,'Topic','/traj_vel_cmd2');
msgs(4) = select(bag,'Topic','/traj_vel_cmd3');
a1_cmd_timestamp = cell(4, 1);a1_cmd_vel = cell(4, 1);a1_t_start = zeros(4, 1);
for i = 1:motor_num
    % ��ȡ����
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % ��ȡʱ���
    a1_cmd_timestamp{i} = msgs(i).MessageList.Time;
    % ��ȡĿ���ٶ�
    a1_cmd_vel{i} = zeros(data_num, 1);
    for j = 1:data_num
        a1_cmd_vel{i}(j) = data{j}.Data;
    end
    % �޸�ʱ���
    a1_t_start(i) = a1_cmd_timestamp{i}(1);
    a1_cmd_timestamp{i}(:) = a1_cmd_timestamp{i}(:) - a1_t_start(i);  
end
%% ��ȡA1��ǰ�ٶ�
% msgs(1) = select(bag,'Topic','/traj_vel_state0');
% msgs(2) = select(bag,'Topic','/traj_vel_state1');
% msgs(3) = select(bag,'Topic','/traj_vel_state2');
% msgs(4) = select(bag,'Topic','/traj_vel_state3');
msgs(1) = select(bag,'Topic','/LW0');
msgs(2) = select(bag,'Topic','/LW1');
msgs(3) = select(bag,'Topic','/LW2');
msgs(4) = select(bag,'Topic','/LW3');
a1_cur_timestamp = cell(4, 1);a1_cur_vel = cell(4, 1);
for i = 1:motor_num
    % ��ȡ����
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % ��ȡʱ���
    a1_cur_timestamp{i} = msgs(i).MessageList.Time;
    % ��ȡ��ǰ�ٶ�
    a1_cur_vel{i} = zeros(data_num, 1);
    for j = 1:data_num
        a1_cur_vel{i}(j) = data{j}.Data;
    end
    % �޸�ʱ���
    a1_cur_timestamp{i}(:) = a1_cur_timestamp{i}(:) - a1_t_start(i);   
end
%% ����A1�ٶ����
a1_error = cell(4, 1); a1_error_max = zeros(4, 1); 
for i = 1:motor_num
    num = size(a1_cmd_timestamp{i},1);
    for j = 1:num
        [~,index] = min(abs(a1_cur_timestamp{i} - a1_cmd_timestamp{i}(j)));
        a1_error{i}(j) = a1_cmd_vel{i}(j)-a1_cur_vel{i}(index);
    end
end

for i = 1:motor_num
    a1_error_max(i) = max(a1_error{i});
end
%% ��ͼ
figure(1);
for i = 1:motor_num
    subplot(2,2,i);
    yyaxis left; % ���������������
    plot(a1_cmd_timestamp{i},a1_cmd_vel{i},'r-',...
         a1_cur_timestamp{i},a1_cur_vel{i},'b--','LineWidth',1.2);
    ylabel('vel(m/s)');
    xlabel('time(s)');xlim([0 17]);
    yyaxis right; % �����Ҳ���������
    plot(a1_cmd_timestamp{i},a1_error{i},'-.','LineWidth',1.2);
    ylabel('error(m/s)');
    legend('target vel','current vel','vel error','FontSize',5,'EdgeColor','none');
    title(['a1', ' vel analysis', num2str(i-1)],'FontSize',10);
end
% savefig([file, '\a1_error.fig'])
%% ��ȡgoĿ��λ��
msgs(1) = select(bag,'Topic','/traj_angle_cmd0');
msgs(2) = select(bag,'Topic','/traj_angle_cmd1');
msgs(3) = select(bag,'Topic','/traj_angle_cmd2');
msgs(4) = select(bag,'Topic','/traj_angle_cmd3');
go_cmd_timestamp = cell(4, 1);go_cmd_angle = cell(4, 1);go_t_start = zeros(4, 1);
for i = 1:motor_num
    % ��ȡ����
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % ��ȡʱ���
    go_cmd_timestamp{i} = msgs(i).MessageList.Time;
    % ��ȡĿ��λ��
    go_cmd_angle{i} = zeros(data_num, 1);
    for j = 1:data_num
        go_cmd_angle{i}(j) = data{j}.Data;
    end
    % �޸�ʱ���
    go_t_start(i) = go_cmd_timestamp{i}(1);
    go_cmd_timestamp{i}(:) = go_cmd_timestamp{i}(:) - go_t_start(i);  
end
%% ��ȡgo��ǰλ��
msgs(1) = select(bag,'Topic','/traj_angle_state0');
msgs(2) = select(bag,'Topic','/traj_angle_state1');
msgs(3) = select(bag,'Topic','/traj_angle_state2');
msgs(4) = select(bag,'Topic','/traj_angle_state3');
go_cur_timestamp = cell(4, 1);go_cur_angle = cell(4, 1);
for i = 1:motor_num
    % ��ȡ����
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % ��ȡʱ���
    go_cur_timestamp{i} = msgs(i).MessageList.Time;
    % ��ȡ��ǰλ��
    go_cur_angle{i} = zeros(data_num, 1);
    for j = 1:data_num
        go_cur_angle{i}(j) = data{j}.Data;
    end
    % �޸�ʱ���
    go_cur_timestamp{i}(:) = go_cur_timestamp{i}(:) - go_t_start(i);   
end
%% ����goλ�����
go_error = cell(4, 1); go_error_max = zeros(4, 1); 
for i = 1:motor_num
    num = size(go_cmd_timestamp{i},1);
    for j = 1:num
        [~,index] = min(abs(go_cur_timestamp{i} - go_cmd_timestamp{i}(j)));
        go_error{i}(j) = go_cmd_angle{i}(j)-go_cur_angle{i}(index);
    end
end

for i = 1:motor_num
    go_error_max(i) = max(go_error{i});
end
%% ��ͼ
figure(2);
for i = 1:motor_num
    subplot(2,2,i);
    yyaxis left; % ���������������
    plot(go_cmd_timestamp{i},go_cmd_angle{i},'r-',...
         go_cur_timestamp{i},go_cur_angle{i},'b--','LineWidth',1.2);
    ylabel('angle(rad)');
    xlabel('time(s)');xlim([0 17]);
    yyaxis right; % �����Ҳ���������
    plot(go_cmd_timestamp{i},go_error{i},'-.','LineWidth',1.2);
    ylabel('error(rad)');
    legend('target angle','current angle','angle error','FontSize',5,'EdgeColor','none');
    title(['go', ' angle analysis', num2str(i-1)],'FontSize',10);
end
% savefig([file, '\go_error.fig'])