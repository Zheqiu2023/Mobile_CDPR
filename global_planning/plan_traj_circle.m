% �滮Բ�켣
function [t_vec,plan_result] = plan_traj_circle(param_traj,plan_result_last,param_cdpr)
t_start = param_traj{1}; t_step = param_traj{2}; t_end = param_traj{3};
p_start = param_traj{4}; p_center = param_traj{5}; p_end = param_traj{6};
total_time = t_end-t_start;     % ������ʱ��
t_vec = 0:t_step:total_time;
t_num = length(t_vec);

start_x=p_start(1); start_y=p_start(2); start_z=p_start(3);
c_x=p_center(1); c_y=p_center(2); c_z=p_center(3);% Բ��
end_x=p_end(1); end_y=p_end(2); end_z=p_end(3);
%% ��Բ�켣�����й켣���xyz����
A_start = angle((start_x-c_x) + (start_y-c_y)*1i);  % -pi~pi
A_end = angle((end_x-c_x)+(end_y-c_y)*1i);
if(A_end<=A_start)
    A_end=A_end+2*pi;
end
% �Ƕȹ滮
A_vec = traj_generator.QuinticInterpolation(A_start, 0, 0, A_end, 0, 0, t_vec);

% ��xyz����
radius=norm([start_x,start_y,start_z]-[c_x,c_y,c_z]);
traj_x=radius*cos(A_vec)+c_x; 
traj_y=radius*sin(A_vec)+c_y; 
traj_z=0*ones(1,t_num)+c_z;
% ���滮���д���ļ����Ժ���������Ա�
writematrix(traj_x,'test.xlsx','WriteMode','append');   % ����ĩ��ƽ̨����λ�ã�������̬��Ϊ[0;0;0]
writematrix(traj_y,'test.xlsx','WriteMode','append');
writematrix(traj_z,'test.xlsx','WriteMode','append');
fid1 = fopen('ee_target_pose.csv', 'a', 'n', 'UTF-8');
fid2 = fopen('test.csv', 'a', 'n', 'UTF-8');    % ����һ��csv�ļ�,׷�����ݵ��ļ�ĩβ
%% ��ʵ��λ��
traj_Ax = zeros(1,t_num); traj_Ay = zeros(1,t_num); traj_Az = zeros(1,t_num);
plan_result = zeros(21,t_num);
for i=1:t_num
    pose_traj = [traj_x(i);traj_y(i);traj_z(i);traj_Ax(i);traj_Ay(i);traj_Az(i)];
    [real_z, ~, real_cl, real_pose_cdpr] = inverse_kine(pose_traj, plan_result_last, param_cdpr);
    [real_pose, real_cf] = direct_kine(real_z, real_cl, real_pose_cdpr, plan_result_last, param_cdpr);

    steer_angle = atan2(real_pose_cdpr(2)-plan_result_last(20), real_pose_cdpr(1)-plan_result_last(19)); % ����ת��Ƕȣ�rad
    roll_angle = sqrt(sumsqr(real_pose_cdpr(1:2)-plan_result_last(19:20)))/param_cdpr.wheel_radius; % ���ֹ�ת�Ƕȣ�rad

    plan_result(:,i) = [real_z;real_cf;real_cl;real_pose;real_pose_cdpr]; 
    plan_result_last = [real_z;real_cf;real_cl;real_pose;real_pose_cdpr];% ������֤���ڹ켣��������������

    % ���˶�ѧ��������λ�ˣ�д���ļ��У���ʵ��λ�˶Ա�
    fprintf(fid1, '%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n', real_pose);    % һ��6�����ݣ��ö��ŷָ���ÿ�н��������\n����
    % �����д��.csv�ļ����������Ƶ������
    fprintf(fid2, '%.5f,', real_z-param_cdpr.bp_z_init);    % һ��10�����ݣ��ö��ŷָ���ÿ�н��������\n���� 
    fprintf(fid2, '%.5f,%.5f,%.5f,%.5f,', real_cl-param_cdpr.cl_init);
    fprintf(fid2, '%.5f,%.5f\n', steer_angle, roll_angle);
end

% �������д���ļ����Ա�鿴
fclose(fid1); 
fclose(fid2);
writematrix(plan_result(13:18,:),'test.xlsx','WriteMode','append');   % ����ĩ��ƽ̨λ��
writematrix(plan_result(19:21,:),'test.xlsx','Sheet',2,'WriteMode','append');   % ����cdprλ��
writematrix(plan_result(1:4,:),'test.xlsx','Sheet',3,'WriteMode','append');   % ����Z����
writematrix(plan_result(9:12,:),'test.xlsx','Sheet',4,'WriteMode','append');   % ��������
writematrix(plan_result(5:8,:),'test.xlsx','Sheet',5,'WriteMode','append');   % ��������