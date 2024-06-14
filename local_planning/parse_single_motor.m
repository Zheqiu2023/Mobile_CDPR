clear, close all
%% 读取文件
fid1 = fopen('data/cablePos06_07_23_03_39.csv');
fid2 = fopen('data/recvPos06_07_23_03_39.csv');
A = textscan(fid1, '%f64%f64%f64%f64', 'Delimiter', ',');
B = textscan(fid2, '%f64%f64%f64%f64', 'Delimiter', ',');
fclose(fid1); fclose(fid2);
a = cell2mat(A);
b = cell2mat(B);
%% 将各个锚点座和绳索的数据单独提取出来
target_pos = [a(:,1), a(:,3)];
target_vel = [a(:,1), -a(:,4)];

cur_pos = [b(:,1), b(:,3)];
cur_vel = [b(:,1), b(:,4)];

% 同一时刻的数据只保留最后一个
k = 1;
n = 1;
for j = 2:size(cur_vel, 1)
    if cur_vel(j,1) == cur_vel(k,1)
        cur_vel(k,2) = cur_vel(j,2);
    else
        k = k + 1;
        cur_vel(k,:) = cur_vel(j,:);
    end
end
for m = 2:size(cur_pos, 1)
    if cur_pos(m,1) == cur_pos(n,1)
        cur_pos(n,2) = cur_pos(m,2);
    else
        n = n + 1;
        cur_pos(n,:) = cur_pos(m,:);
    end
end
cur_vel = cur_vel(1:k,:);
cur_pos = cur_pos(1:n,:);

%% 修改时间戳
vel_t_start = target_vel (1,1);
for j = 1:size(target_vel , 1)
    target_vel (j,1) = (target_vel (j,1) - vel_t_start) / 1000;
end
for j = 1:size(cur_vel , 1)
    cur_vel (j,1) = (cur_vel (j,1) - vel_t_start) / 1000;
end
pos_t_start = target_pos (1,1);
for j = 1:size(target_pos , 1)
    target_pos (j,1) = (target_pos (j,1) - pos_t_start) / 1000;
end
for j = 1:size(cur_pos , 1)
    cur_pos (j,1) = (cur_pos (j,1) - pos_t_start) / 1000;
end
%% 计算电机位置误差
for j = 1:size(target_vel , 1)
    [~,index] = min(abs(cur_vel (:, 1) - target_vel (j, 1)));
    vel_error (j) = target_vel (j, 2) - cur_vel (index, 2);
end
for j = 1:size(target_pos , 1)
    [~,index] = min(abs(cur_pos (:, 1) - target_pos (j, 1)));
    pos_error (j) = target_pos (j, 2) - cur_pos (index, 2);
end
vel_error_max = max(vel_error );
pos_error_max = max(pos_error );

%% 绘图
% 锚点座
figure(1);
yyaxis left; % 设置左侧纵坐标轴
plot(target_vel (:,1),target_vel (:,2),'r-',...
    cur_vel (:,1),cur_vel (:,2),'b--','LineWidth',1.2);
ylabel('pos(rpm)');
xlabel('time(s)');xlim([0 12]);
yyaxis right; % 设置右侧纵坐标轴
plot(target_vel (:,1),vel_error ,'-.','LineWidth',1.2);
ylabel('error(rpm)');
legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none');
title(['vel analysis'],'FontSize',10);

% 绳索
figure(2);
yyaxis left; % 设置左侧纵坐标轴
plot(target_pos (:,1),target_pos (:,2),'r-',...
    cur_pos (:,1),cur_pos (:,2),'b--','LineWidth',1.2);
ylabel('pos(°)');
xlabel('time(s)');xlim([0 12]);
yyaxis right; % 设置右侧纵坐标轴
plot(target_pos (:,1),pos_error ,'-.','LineWidth',1.2);
ylabel('error(°)');
legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none','Location','southeast');
title(['pos analysis'],'FontSize',10);
