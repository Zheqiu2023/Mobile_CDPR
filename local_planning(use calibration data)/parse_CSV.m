clear, close all
param_cdpr = param_mobile_cdpr;
%% 读取文件
fid1 = fopen('data/archorPos04_14_14_18_14.csv');   
fid2 = fopen('data/cablePos04_14_14_18_14.csv');
fid3 = fopen('data/recvPos04_14_14_18_14.csv');
fid4 = fopen('data/ee_target_pose04_14_14_18_14.csv');
A = textscan(fid1, '%f64%f64%f64', 'Delimiter', ',');
B = textscan(fid2, '%f64%f64%f64', 'Delimiter', ',');
C = textscan(fid3, '%f64%f64%f64', 'Delimiter', ',');
D = textscan(fid4, '%f64%f64%f64%f64%f64%f64', 'Delimiter', ',');
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
    archor_target_pos{i}(:,1) = (archor_target_pos{i}(:,1) - archor_t_start) / 1000;
    archor_cur_pos{i}(:,1) = (archor_cur_pos{i}(:,1) - archor_t_start) / 1000;
    cable_t_start = cable_target_pos{i}(1,1);
    cable_target_pos{i}(:,1) = (cable_target_pos{i}(:,1) - cable_t_start) / 1000;
    cable_cur_pos{i}(:,1) = (cable_cur_pos{i}(:,1) - cable_t_start) / 1000;
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
    archor_error_max(i) = max(archor_error{i});
    cable_error_max(i) = max(cable_error{i});
end
%% 绘图
% 锚点座
figure(1);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(archor_target_pos{i}(:,1),archor_target_pos{i}(:,2),'r-',...
         archor_cur_pos{i}(:,1),archor_cur_pos{i}(:,2),'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 23]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(archor_target_pos{i}(:,1),archor_error{i},'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none');
    title(['archor', num2str(i-1),' pos analysis'],'FontSize',10);
end
% 绳索
figure(2);
for i = 1:4
    subplot(2,2,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(cable_target_pos{i}(:,1),cable_target_pos{i}(:,2),'r-',...
         cable_cur_pos{i}(:,1),cable_cur_pos{i}(:,2),'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 23]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(cable_target_pos{i}(:,1),cable_error{i},'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','pos error','FontSize',5,'EdgeColor','none','Location','southeast');
    title(['cable',num2str(i-1),' pos analysis'],'FontSize',10);
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

real_pose = zeros(6,num);target_pose = d';
acp = zeros(4, num);ccp = zeros(4, num);
target_timestamp = archor_target_pos{1}(:,1);

for i = 1:4
    for j = 1:num
        [~,index2] = min(abs(archor_cur_pos{i}(:,1) - (cur_timestamp(j))));
        [~,index3] = min(abs(cable_cur_pos{i}(:,1) - (cur_timestamp(j))));
        acp(i,j) = archor_cur_pos{i}(index2,2) + param_cdpr.init_h(i);
        ccp(i,j) = cable_cur_pos{i}(index3,2) + param_cdpr.cl_init(i);
    end
end
% for i = 1:4
%     for j = 1:num
%         [~,index2] = min(abs(archor_cur_pos{i}(:,1) - (target_timestamp(j))));
%         [~,index3] = min(abs(cable_cur_pos{i}(:,1) - (target_timestamp(j))));
%         acp(i,j) = archor_cur_pos{i}(index2,2) + param_cdpr.bp_z_init(i);
%         ccp(i,j) = cable_cur_pos{i}(index3,2) + param_cdpr.cl_init(i);
%     end
%     archor_target_pos{i}(:,2) = archor_target_pos{i}(:,2) + param_cdpr.bp_z_init(i);
%     cable_target_pos{i}(:,2) = cable_target_pos{i}(:,2) + param_cdpr.cl_init(i);
% end

% 末端平台实际位置、期望位置
real_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.03;0;0;0];
target_pose_last = [zeros(4,1);1;1;1;1;zeros(4,1);0;0;0.03;0;0;0];
for n = 1:num
    [real, ~] = direct_kine(acp(:,n), ccp(:,n), real_pose_last, param_cdpr);
    real_pose(:,n) = real;
    real_pose_last(13:18) = real;
end
% for n = 1:num
%     z = [archor_target_pos{1}(n,2);archor_target_pos{2}(n,2);...
%          archor_target_pos{3}(n,2);archor_target_pos{4}(n,2)];
%     l = [cable_target_pos{1}(n,2);cable_target_pos{2}(n,2);...
%          cable_target_pos{3}(n,2);cable_target_pos{4}(n,2)];
%     [target, ~] = direct_kine(z, l, target_pose_last);
%     target_pose(:,n) = target;
%     target_pose_last(13:18) = target;
% end
%% 计算末端平台位置误差
ep_pos_error = zeros(6, size(target_timestamp, 1)); ep_pos_error_max = zeros(6, 1); 
for j = 1:size(target_timestamp, 1)
    [~,index] = min(abs(cur_timestamp - target_timestamp(j)));
    ep_pos_error(:, j) = target_pose(:, j) - real_pose(:, index);
end

for i = 1:6
    ep_pos_error_max(i) = max(ep_pos_error(i, :));
end
% 绘图
figure(3);
for i = 1:3
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(target_timestamp,target_pose(i,:),'r-',...
         cur_timestamp,real_pose(i,:),'b--','LineWidth',1.2);
    ylabel('pos(m)');
    xlabel('time(s)');xlim([0 23]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(target_timestamp,ep_pos_error(i,:),'-.','LineWidth',1.2);
    ylabel('error(m)');
    legend('target pos','current pos','FontSize',5,'EdgeColor','none');
    title('end plaform pos analysis','FontSize',10);
end
for i = 4:6
    subplot(2,3,i);
    yyaxis left; % 设置左侧纵坐标轴
    plot(target_timestamp,target_pose(i,:)*180/pi,'r-',...
         cur_timestamp,real_pose(i,:)*180/pi,'b--','LineWidth',1.2);
    ylabel('angle(°)');
    xlabel('time(s)');xlim([0 23]);
    yyaxis right; % 设置右侧纵坐标轴
    plot(target_timestamp,ep_pos_error(i,:)*180/pi,'-.','LineWidth',1.2);
    ylabel('error(°)');
    legend('target angle','current angle','FontSize',5,'EdgeColor','none');
    title('end plaform angle analysis','FontSize',10);
end

