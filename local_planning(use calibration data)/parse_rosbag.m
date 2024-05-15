clear, close all
file = 'updown';
bag = rosbag([file,'/',file,'.bag']);
%% 读取锚点座目标位置
msgs = select(bag,'Topic','/archor_coor_z');
% 读取数据
data = readMessages(msgs,'DataFormat','struct'); 
data_num = size(data,1);
% 读取时间戳
archor_target_timestamp = msgs.MessageList.Time;
% 读取位置
archor_target_pos = zeros(4,data_num);
for i = 1:data_num
    archor_target_pos(:,i) = data{i}.Target;
end
% 修改时间戳
t_start = archor_target_timestamp(1);
for i = 1:data_num
    archor_target_timestamp(i) = archor_target_timestamp(i) - t_start;
end
%% 读取锚点座当前位置
msgs(1) = select(bag,'Topic','/archor0_cur_pos');
msgs(2) = select(bag,'Topic','/archor1_cur_pos');
msgs(3) = select(bag,'Topic','/archor2_cur_pos');
msgs(4) = select(bag,'Topic','/archor3_cur_pos');
archor_cur_timestamp = cell(4, 1);archor_cur_pos = cell(4, 1);
for i = 1:4
    % 读取数据
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % 读取时间戳
    archor_cur_timestamp{i} = msgs(i).MessageList.Time;
    % 读取位置
    archor_cur_pos{i} = zeros(data_num, 1);
    for j = 1:data_num
        archor_cur_pos{i}(j) = data{j}.Data;
    end
    % 修改时间戳（对时间进行补偿）
    t_start = archor_cur_timestamp{i}(1);
    for j = 1:data_num
        % 1.6为估计的时间补偿值(1.6 for updown; 1.5 for line; 1.3 for circle)
        archor_cur_timestamp{i}(j) = archor_cur_timestamp{i}(j) - t_start;  
    end
end
%% 计算锚点座位置误差
num = size(archor_target_timestamp,1);
archor_error = cell(4, 1); archor_error_max = zeros(4, 1); 
for i = 1:num
    for j = 1:4
%         [~,index] = sort(abs(archor_cur_timestamp{j} - archor_target_timestamp(i)));
%         archor_error{j}(i) = min(archor_target_pos(j, i)-archor_cur_pos{j}(index(1:5)));
        [~,index] = min(abs(archor_cur_timestamp{j} - archor_target_timestamp(i)));
        archor_error{j}(i) = archor_target_pos(j, i)-archor_cur_pos{j}(index);
    end
end

for i = 1:4
    archor_error_max(i) = max(archor_error{i});
end
%% 绘图
figure(1);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(archor_target_timestamp,archor_target_pos(i,:),'r-',...
         archor_cur_timestamp{i},archor_cur_pos{i},'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 100]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(archor_target_timestamp,archor_error{i},'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none');
    title(['archor', num2str(i-1),' pos analysis'],'FontSize',10);
end
% savefig([file, '\archor_error.fig'])
%% 读取绳索目标位置
msgs = select(bag,'Topic','/cable_length');
% 读取数据
data = readMessages(msgs,'DataFormat','struct'); 
data_num = size(data,1);
% 读取时间戳
cable_target_timestamp = msgs.MessageList.Time;
% 读取位置
cable_target_pos = zeros(4,data_num);
for i = 1:data_num
    cable_target_pos(:,i) = data{i}.Target;
end
% 修改时间戳
t_start = cable_target_timestamp(1);
for i=1:data_num
    cable_target_timestamp(i) = cable_target_timestamp(i) - t_start;
end
%% 读取绳索当前位置
msgs(1) = select(bag,'Topic','/cable0_cur_pos');
msgs(2) = select(bag,'Topic','/cable1_cur_pos');
msgs(3) = select(bag,'Topic','/cable2_cur_pos');
msgs(4) = select(bag,'Topic','/cable3_cur_pos');
cable_cur_timestamp = cell(4, 1);cable_cur_pos = cell(4, 1);
for i = 1:4
    % 读取数据
    data = readMessages(msgs(i),'DataFormat','struct'); 
    data_num = size(data,1);
    % 读取时间戳
    cable_cur_timestamp{i} = msgs(i).MessageList.Time;
    % 读取位置
    cable_cur_pos{i} = zeros(data_num, 1);
    for j = 1:data_num
        cable_cur_pos{i}(j) = data{j}.Data;
    end
    % 修改时间戳（对时间进行补偿）
    t_start = cable_cur_timestamp{i}(1);
    for j=1:data_num
        % 2.8为估计的时间补偿值 (2.8 for updown; 1 for line; 1.2 for circle)
        cable_cur_timestamp{i}(j) = cable_cur_timestamp{i}(j) - t_start;  
    end
end
%% 计算绳索位置误差
num = size(cable_target_timestamp,1);
cable_error = cell(4, 1); cable_error_max = zeros(4, 1); 
for i = 1:num
    for j = 1:4
%         [~,index] = sort(abs(cable_cur_timestamp{j} - cable_target_timestamp(i)));
%         cable_error{j}(i) = min(cable_target_pos(j, i)-cable_cur_pos{j}(index(1:5)));
        [~,index] = min(abs(cable_cur_timestamp{j} - cable_target_timestamp(i)));
        cable_error{j}(i) = cable_target_pos(j, i)-cable_cur_pos{j}(index);
    end
end

for i = 1:4
    cable_error_max(i) = max(cable_error{i});
end
%% 绘图
figure(2);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cable_target_timestamp,cable_target_pos(i,:),'r-',...
         cable_cur_timestamp{i},cable_cur_pos{i},'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 100]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cable_target_timestamp,cable_error{i},'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none','Location','southeast');
    title(['cable',num2str(i-1),' pos analysis'],'FontSize',10);
end
% savefig([file, '\cable_error.fig'])
%% 正运动学求解末端平台实际位置，与期望位置做对比
[num1,index] = min([size(archor_cur_pos{1},1),size(archor_cur_pos{2},1),...
           size(archor_cur_pos{3},1),size(archor_cur_pos{4},1),...
           size(cable_cur_pos{1},1),size(cable_cur_pos{2},1),...
           size(cable_cur_pos{3},1),size(cable_cur_pos{4},1)]);
acp = zeros(4, num1);ccp = zeros(4, num1);
for i = 1:4
    acp(i,:) = archor_cur_pos{i}(1:num1);
    ccp(i,:) = cable_cur_pos{i}(1:num1);
end
if index < 5
    timestamp = archor_cur_timestamp{index}; 
else
    timestamp = cable_cur_timestamp{index-4}; 
end
num2 = min(size(archor_target_pos,2),size(cable_target_pos,2));
real_pose = zeros(6,num1); 
target_pose = zeros(6,num2);
% 末端平台实际位置
pose_last = [0;0;30e-3;0;0;0];
for n = 1:num1
    [pose, ~] = direct_kine(acp(:,n), ccp(:,n), pose_last);
    real_pose(:,n) = pose;
    pose_last = pose;
end
% 末端平台期望位置
pose_last = [0;0;30e-3;0;0;0];
for n = 1:num2
    [pose, ~] = direct_kine(archor_target_pos(:,n), cable_target_pos(:,n), pose_last);
    target_pose(:,n) = pose;
    pose_last = pose;
end
%% 计算末端平台位置误差
ep_pos_error = zeros(6, num2); ep_pos_error_max = zeros(6, 1); 
for i = 1:num2
    [~, index] = min(abs(timestamp - archor_target_timestamp(i)));
    ep_pos_error(:, i) = target_pose(:, i) - real_pose(:, index);
end

for i = 1:6
    ep_pos_error_max(i) = max(ep_pos_error(i, :));
end
% 绘图
figure(3);
for i = 1:3
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(archor_target_timestamp,target_pose(i,:),'r-',...
         timestamp,real_pose(i,:),'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 100]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(archor_target_timestamp,ep_pos_error(i,:),'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','FontSize',5,'EdgeColor','none');
    title('end plaform pos analysis','FontSize',10);
end
for i = 4:6
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(archor_target_timestamp,target_pose(i,:),'r-',...
         timestamp,real_pose(i,:),'b--','LineWidth',1.2);
    ylabel('angle(rad)');
    xlabel('time(s)');xlim([0 100]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(archor_target_timestamp,ep_pos_error(i,:),'-.','LineWidth',1.2);
    ylabel('error(rad)');
    legend('target angle','current angle','FontSize',5,'EdgeColor','none');
    title('end plaform angle analysis','FontSize',10);
end
