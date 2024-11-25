clear, close all
param_cdpr = param_mobile_cdpr;
%% 读取文件
fid1 = fopen('data/archorPos03_18_16_24_15.csv');   
fid2 = fopen('data/cablePos03_18_16_24_15.csv');
fid3 = fopen('data/recvPos03_18_16_24_15.csv');
fid4 = fopen('data/03_08_21_45_10.csv');
A = textscan(fid1, '%f64%f64%f64', 'Delimiter', ',');
B = textscan(fid2, '%f64%f64%f64', 'Delimiter', ',');
C = textscan(fid3, '%f64%f64%f64', 'Delimiter', ',');
D = textscan(fid4, '%f64%f64%f64%f64%f64%f64%f64%f64', 'Delimiter', ',');
fclose(fid1); fclose(fid2); fclose(fid3); fclose(fid4);
a = cell2mat(A);
b = cell2mat(B);
c = cell2mat(C);
d = cell2mat(D);
%% 将各个锚点座和绳索的数据单独提取出来
archor_target_pos = cell(4, 1); archor_cur_pos = cell(4, 1);
cable_target_pos = cell(4, 1); cable_cur_pos = cell(4, 1);

for i = 1:size(a, 1)
    n = a(i, 2);
    archor_target_pos{n-4} = vertcat(archor_target_pos{n-4},[a(i,1),a(i,3)]);
end
for i = 1:size(b, 1)
    n = b(i, 2);
    cable_target_pos{n} = vertcat(cable_target_pos{n},[b(i,1),b(i,3)]);
end
for i = 1:size(c, 1)
    n = c(i, 2);
    if(n>=1&&n<=4)
        cable_cur_pos{n} = vertcat(cable_cur_pos{n},[c(i,1),c(i,3)]);
    elseif(n>=5&&n<=8)
        archor_cur_pos{n-4} = vertcat(archor_cur_pos{n-4},[c(i,1),c(i,3)]);
    end
end
% 同一时刻的数据只保留最后一个
for i = 1:4
    k = 1;
    n = 1;
    for j = 2:size(archor_cur_pos{i}, 1)
        if archor_cur_pos{i}(j,1) == archor_cur_pos{i}(k,1)
            archor_cur_pos{i}(k,2) = archor_cur_pos{i}(j,2);
        else
            k = k + 1;
            archor_cur_pos{i}(k,:) = archor_cur_pos{i}(j,:);
        end
    end
    for m = 2:size(cable_cur_pos{i}, 1)
        if cable_cur_pos{i}(m,1) == cable_cur_pos{i}(n,1)
            cable_cur_pos{i}(n,2) = cable_cur_pos{i}(m,2);
        else
            n = n + 1;
            cable_cur_pos{i}(n,:) = cable_cur_pos{i}(m,:);
        end
    end
    archor_cur_pos{i} = archor_cur_pos{i}(1:k,:);
    cable_cur_pos{i} = cable_cur_pos{i}(1:n,:);
end

%% 修改时间戳
for i = 1:4
    archor_t_start = archor_target_pos{i}(1,1);
    for j = 1:size(archor_target_pos{i}, 1)
        archor_target_pos{i}(j,1) = (archor_target_pos{i}(j,1) - archor_t_start) / 1000;
    end
    for j = 1:size(archor_cur_pos{i}, 1)
        archor_cur_pos{i}(j,1) = (archor_cur_pos{i}(j,1) - archor_t_start) / 1000;
    end
    cable_t_start = cable_target_pos{i}(1,1);
    for j = 1:size(cable_target_pos{i}, 1)
        cable_target_pos{i}(j,1) = (cable_target_pos{i}(j,1) - cable_t_start) / 1000;
    end
    for j = 1:size(cable_cur_pos{i}, 1)
        cable_cur_pos{i}(j,1) = (cable_cur_pos{i}(j,1) - cable_t_start) / 1000;
    end
end
%% 计算电机位置误差
archor_error = cell(4, 1); archor_error_max = zeros(4, 1);
cable_error = cell(4, 1); cable_error_max = zeros(4, 1);
for i = 1:4
    for j = 1:size(archor_target_pos{i}, 1)
        [~,index] = min(abs(archor_cur_pos{i}(:, 1) - archor_target_pos{i}(j, 1)));
        archor_error{i}(j) = archor_target_pos{i}(j, 2) - archor_cur_pos{i}(index, 2);
    end
    for j = 1:size(cable_target_pos{i}, 1)
        [~,index] = min(abs(cable_cur_pos{i}(:, 1) - cable_target_pos{i}(j, 1)));
        cable_error{i}(j) = cable_target_pos{i}(j, 2) - cable_cur_pos{i}(index, 2);
    end
    archor_error_max(i) = max(abs(archor_error{i}));
    cable_error_max(i) = max(abs(cable_error{i}));
end
%% 绘图
% 锚点座
figure(1);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(archor_target_pos{i}(:,1),archor_target_pos{i}(:,2),'r-',...
         archor_cur_pos{i}(:,1),archor_cur_pos{i}(:,2),'b--','LineWidth',1.2);
    ylabel('位置(m)');
    xlabel('时间(s)');xlim([0 220]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(archor_target_pos{i}(:,1),archor_error{i},'-.','LineWidth',1.2);
    ylabel('误差(m)');
    legend('期望位置','实际位置','位置误差','FontSize',5,'EdgeColor','none');
    title(['锚点座', num2str(i)],'FontSize',10);
end
% 绳索
figure(2);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cable_target_pos{i}(:,1),cable_target_pos{i}(:,2),'r-',...
         cable_cur_pos{i}(:,1),cable_cur_pos{i}(:,2),'b--','LineWidth',1.2);
    ylabel('位置(m)');
    xlabel('时间(s)');xlim([0 220]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cable_target_pos{i}(:,1),cable_error{i},'-.','LineWidth',1.2);
    ylabel('误差(m)');
    legend('期望位置','实际位置','位置误差','FontSize',5,'EdgeColor','none','Location','southeast');
    title(['绳索',num2str(i)],'FontSize',10);
end
%% 正运动学求解末端平台实际位置，与期望位置做对比
num = size(archor_cur_pos{1},1);
cur_timestamp = archor_cur_pos{1}(:,1);
for i = 1:4
    a = size(archor_cur_pos{i},1);
    b = size(cable_cur_pos{i},1);
    if a < num 
        num = a; 
        cur_timestamp = archor_cur_pos{i}(:,1);
    end
    if b < num 
        num = b;
        cur_timestamp = cable_cur_pos{i}(:,1);
    end
end

real_pose = zeros(6,num);
acp = zeros(4, num);ccp = zeros(4, num);
target_timestamp = archor_target_pos{1}(:,1);

for i = 1:4
    for j = 1:num
        [~,index2] = min(abs(archor_cur_pos{i}(:,1) - (cur_timestamp(j))));
        [~,index3] = min(abs(cable_cur_pos{i}(:,1) - (cur_timestamp(j))));
        acp(i,j) = archor_cur_pos{i}(index2,2) + param_cdpr.bp_z_init(i);
        ccp(i,j) = cable_cur_pos{i}(index3,2) + param_cdpr.cl_init(i);
    end
end

% 末端平台实际位置、期望位置
real_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.028;0;0;0];
target_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.028;0;0;0];
for n = 1:size(d, 1)
    archor = d(n, 1:4)' + param_cdpr.bp_z_init;
    cable = d(n, 5:8)' + param_cdpr.cl_init;
    [res, ~] = direct_kine(archor, cable, target_pose_last, param_cdpr)
    target_pose(:,n) = res;
    target_pose_last(13:18) = res;
end

for n = 1:num
    [res, ~] = direct_kine(acp(:,n), ccp(:,n), real_pose_last, param_cdpr)
    real_pose(:,n) = res;
    real_pose_last(13:18) = res;
end

%% 计算末端平台位置误差
ep_pos_error = zeros(6, size(target_timestamp, 1)); ep_pos_error_max = zeros(6, 1); 
for j = 1:size(target_timestamp, 1)
    [~,index] = min(abs(cur_timestamp - target_timestamp(j)));
    ep_pos_error(:, j) = target_pose(:, j) - real_pose(:, index);
end

for i = 1:6
    ep_pos_error_max(i) = max(ep_pos_error(i, :));
end

%% 绘图
figure(3);
set(gcf,'color','white');
for i = 1:3
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(target_timestamp,target_pose(i,:),'r-',...
         cur_timestamp,real_pose(i,:),'b--','LineWidth',1.2);
    ylabel('位置(m)', 'FontSize', 12);
    xlabel('时间(s)', 'FontSize', 12);xlim([0 22]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(target_timestamp,ep_pos_error(i,:),'g-.','LineWidth',1.2,'Color',[0.7, 0.5, 0.3]);
    ylabel('误差(m)');
    legend('期望位置','实际位置','位置误差','FontSize',10,'EdgeColor','none','Location','northeast');
    if i == 1
        title('x轴位置', 'FontSize', 12);
    elseif i == 2
        title('y轴位置', 'FontSize', 12);
    elseif i == 3
        title('z轴位置', 'FontSize', 12);
    end
end
for i = 4:6
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(target_timestamp,target_pose(i,:)*180/pi,'r-',...
         cur_timestamp,real_pose(i,:)*180/pi,'b--','LineWidth',1.2);
    ylabel('角度(°)', 'FontSize', 12);
    xlabel('时间(s)', 'FontSize', 12);xlim([0 22]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(target_timestamp,ep_pos_error(i,:)*180/pi,'g-.','LineWidth',1.2,'Color',[0.7, 0.5, 0.3]);
    ylabel('误差(°)');
    legend('期望角度','实际角度','角度误差','FontSize',10,'EdgeColor','none','Location','northeast');
    if i == 4
        title('x轴姿态', 'FontSize', 12);
    elseif i == 5
        title('y轴姿态', 'FontSize', 12);
    elseif i == 6
        title('z轴姿态', 'FontSize', 12);
    end
end

figure(4);
set(gcf,'color','white');
plot3(target_pose(1,:), target_pose(2,:), target_pose(3,:), 'r-','LineWidth',2);hold on;
plot3(real_pose(1,:), real_pose(2,:), real_pose(3,:), 'b--','LineWidth',2);
xlabel('X(m)', 'FontSize', 13);
ylabel('Y(m)', 'FontSize', 13);
zlabel('Z(m)', 'FontSize', 13);
axis([-0.5 0.5 -0.5 0.5 0 0.7]);
legend('期望轨迹','实际轨迹','FontSize',13,'EdgeColor','none','Location','northeast');


