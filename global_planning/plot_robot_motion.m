function [] = plot_robot_motion(t_vec, pose_ee, pose_cdpr, bp_z, param_cdpr)
trj_x = pose_ee(1,:);
trj_y = pose_ee(2,:);
trj_z = pose_ee(3,:);
%% 获取屏幕大小，scrsz是一个4元素向量[left, bottom, width, height]，然后用获得的scrsz向量设置fig的position属性
figure(1)
scrsz = get(0,'ScreenSize');  
set(gcf,'Position',[0 0 scrsz(3) scrsz(4)-90]);    

%% 绘制锚点座运动范围
% 基坐标系到世界坐标系的齐次变换矩阵
Trans_bw=transl(pose_cdpr(1),pose_cdpr(2),0)*trotz(pose_cdpr(3));
% 边界的8个顶点
v1_w=Trans_bw*param_cdpr.bp_coor(:,1);
v2_w=Trans_bw*param_cdpr.bp_coor(:,2);
v3_w=Trans_bw*param_cdpr.bp_coor(:,3);
v4_w=Trans_bw*param_cdpr.bp_coor(:,4);
v5_w=Trans_bw*[param_cdpr.bp_coor(1,1);param_cdpr.bp_coor(2,1);0.535;1];
v6_w=Trans_bw*[param_cdpr.bp_coor(1,2);param_cdpr.bp_coor(2,2);0.535;1];
v7_w=Trans_bw*[param_cdpr.bp_coor(1,3);param_cdpr.bp_coor(2,3);0.535;1];
v8_w=Trans_bw*[param_cdpr.bp_coor(1,4);param_cdpr.bp_coor(2,4);0.535;1];
v_w=[v1_w,v2_w,v4_w,v3_w,v1_w,v5_w,v6_w,v2_w,v6_w,v8_w,v4_w,v8_w,v7_w,v3_w,v7_w,v5_w];  % 运动范围向量
boundary_frame=plot3(v_w(1,:),v_w(2,:),v_w(3,:),'r','Linewidth',1);
hold on
xlabel('X');ylabel('Y');zlabel('Z');view(30,30);
%% 绘制机器人模型
% 末端执行器坐标系到世界坐标系的齐次变换矩阵
Trans_ew=transl(pose_ee(1),pose_ee(2),pose_ee(3))*trotz(pose_ee(6))*troty(pose_ee(5))*trotx(pose_ee(4));
% 机器人模型为长方体，e1~e8为8个顶点
e1_w=Trans_ew*param_cdpr.ep_coor(:,1);
e2_w=Trans_ew*param_cdpr.ep_coor(:,2);
e3_w=Trans_ew*param_cdpr.ep_coor(:,3);
e4_w=Trans_ew*param_cdpr.ep_coor(:,4);
e5_w=Trans_ew*[param_cdpr.ep_coor(1,1);param_cdpr.ep_coor(2,1);-param_cdpr.ep_coor(3,1);1];
e6_w=Trans_ew*[param_cdpr.ep_coor(1,2);param_cdpr.ep_coor(2,2);-param_cdpr.ep_coor(3,2);1];
e7_w=Trans_ew*[param_cdpr.ep_coor(1,3);param_cdpr.ep_coor(2,3);-param_cdpr.ep_coor(3,3);1];
e8_w=Trans_ew*[param_cdpr.ep_coor(1,4);param_cdpr.ep_coor(2,4);-param_cdpr.ep_coor(3,4);1];

e_w=[e1_w,e2_w,e4_w,e3_w,e1_w,e5_w,e6_w,e2_w,e6_w,e8_w,e4_w,e8_w,e7_w,e3_w,e7_w,e5_w];  % 机器人框架向量
robot_frame=plot3(e_w(1,:),e_w(2,:),e_w(3,:),'k','Linewidth',1);
robot_fill=fill3([e5_w(1,1),e6_w(1,1),e8_w(1,1),e7_w(1,1)],...
                 [e5_w(2,1),e6_w(2,1),e8_w(2,1),e7_w(2,1)],...
                 [e5_w(3,1),e6_w(3,1),e8_w(3,1),e7_w(3,1)],'g');    % 底面
robot_fill2=fill3([e1_w(1,1),e5_w(1,1),e6_w(1,1),e2_w(1,1)],...
                  [e1_w(2,1),e5_w(2,1),e6_w(2,1),e2_w(2,1)],...
                  [e1_w(3,1),e5_w(3,1),e6_w(3,1),e2_w(3,1)],'y');
robot_fill3=fill3([e1_w(1,1),e5_w(1,1),e7_w(1,1),e3_w(1,1)],...
                  [e1_w(2,1),e5_w(2,1),e7_w(2,1),e3_w(2,1)],...
                  [e1_w(3,1),e5_w(3,1),e7_w(3,1),e3_w(3,1)],'y');
robot_fill4=fill3([e3_w(1,1),e7_w(1,1),e8_w(1,1),e4_w(1,1)],...
                  [e3_w(2,1),e7_w(2,1),e8_w(2,1),e4_w(2,1)],...
                  [e3_w(3,1),e7_w(3,1),e8_w(3,1),e4_w(3,1)],'y');
robot_fill5=fill3([e4_w(1,1),e8_w(1,1),e6_w(1,1),e2_w(1,1)],...
                  [e4_w(2,1),e8_w(2,1),e6_w(2,1),e2_w(2,1)],...
                  [e4_w(3,1),e8_w(3,1),e6_w(3,1),e2_w(3,1)],'y');
robot_fill6=fill3([e4_w(1,1),e3_w(1,1),e1_w(1,1),e2_w(1,1)],...
                  [e4_w(2,1),e3_w(2,1),e1_w(2,1),e2_w(2,1)],...
                  [e4_w(3,1),e3_w(3,1),e1_w(3,1),e2_w(3,1)],'g');   % 顶面
%% 绘制绳索
cable_length=[v1_w,e1_w,v2_w,e2_w,v3_w,e3_w,v4_w,e4_w];
cl1=plot3(cable_length(1,1:2),cable_length(2,1:2),cable_length(3,1:2),'k');
cl2=plot3(cable_length(1,3:4),cable_length(2,3:4),cable_length(3,3:4),'k');
cl3=plot3(cable_length(1,5:6),cable_length(2,5:6),cable_length(3,5:6),'k');
cl4=plot3(cable_length(1,7:8),cable_length(2,7:8),cable_length(3,7:8),'k');
%% 绘制世界坐标系原点
scatter3(0,0,0,50,'Marker','o','MarkerEdgeColor','r','MarkerFaceColor','r');
%% 绘制绳索起始点
cable_start_point=scatter3(v_w(1,1:4),v_w(2,1:4),v_w(3,1:4),200,'g','.');

%% 机器人运动动画
% hp = plot3(trj_x(1:2),trj_y(1:2),trj_z(1:2),'Color','b','LineStyle','-','.','MarkerSize',6,...
%     'MarkerEdgeColor','r','MarkerFaceColor','r');
hp=animatedline('Color','b','LineStyle','-');
% axis equal
axis([-1 4 -1 4 0 1.5])
grid on
pause(0.5)

bp_coor = param_cdpr.bp_coor;
for i=1:length(trj_x)
    Trans_bw=transl(pose_cdpr(1,i),pose_cdpr(2,i),0)*trotz(pose_cdpr(3,i));
    v1_w=Trans_bw*param_cdpr.bp_coor(:,1);
    v2_w=Trans_bw*param_cdpr.bp_coor(:,2);
    v3_w=Trans_bw*param_cdpr.bp_coor(:,3);
    v4_w=Trans_bw*param_cdpr.bp_coor(:,4);
    v5_w=Trans_bw*[param_cdpr.bp_coor(1,1);param_cdpr.bp_coor(2,1);0.535;1];
    v6_w=Trans_bw*[param_cdpr.bp_coor(1,2);param_cdpr.bp_coor(2,2);0.535;1];
    v7_w=Trans_bw*[param_cdpr.bp_coor(1,3);param_cdpr.bp_coor(2,3);0.535;1];
    v8_w=Trans_bw*[param_cdpr.bp_coor(1,4);param_cdpr.bp_coor(2,4);0.535;1];
    v_w=[v1_w,v2_w,v4_w,v3_w,v1_w,v5_w,v6_w,v2_w,v6_w,v8_w,v4_w,v8_w,v7_w,v3_w,v7_w,v5_w];
    set(boundary_frame,'Xdata',v_w(1,:),'Ydata',v_w(2,:),'Zdata',v_w(3,:)); 

    Trans_ew=transl(pose_ee(1,i),pose_ee(2,i),pose_ee(3,i))*trotz(pose_ee(6,i))*troty(pose_ee(5,i))*trotx(pose_ee(4,i));
    e1_w=Trans_ew*param_cdpr.ep_coor(:,1);
    e2_w=Trans_ew*param_cdpr.ep_coor(:,2);
    e3_w=Trans_ew*param_cdpr.ep_coor(:,3);
    e4_w=Trans_ew*param_cdpr.ep_coor(:,4);
    e5_w=Trans_ew*[param_cdpr.ep_coor(1,1);param_cdpr.ep_coor(2,1);-param_cdpr.ep_coor(3,1);1];
    e6_w=Trans_ew*[param_cdpr.ep_coor(1,2);param_cdpr.ep_coor(2,2);-param_cdpr.ep_coor(3,2);1];
    e7_w=Trans_ew*[param_cdpr.ep_coor(1,3);param_cdpr.ep_coor(2,3);-param_cdpr.ep_coor(3,3);1];
    e8_w=Trans_ew*[param_cdpr.ep_coor(1,4);param_cdpr.ep_coor(2,4);-param_cdpr.ep_coor(3,4);1];
    e_w=[e1_w,e2_w,e4_w,e3_w,e1_w,e5_w,e6_w,e2_w,e6_w,e8_w,e4_w,e8_w,e7_w,e3_w,e7_w,e5_w];
    set(robot_frame,'Xdata',e_w(1,:),'Ydata',e_w(2,:),'Zdata',e_w(3,:));
    set(robot_fill,'Xdata',[e5_w(1,1),e6_w(1,1),e8_w(1,1),e7_w(1,1)],...
                   'Ydata',[e5_w(2,1),e6_w(2,1),e8_w(2,1),e7_w(2,1)],...
                   'Zdata',[e5_w(3,1),e6_w(3,1),e8_w(3,1),e7_w(3,1)]);
    set(robot_fill2,'Xdata',[e1_w(1,1),e5_w(1,1),e6_w(1,1),e2_w(1,1)],...
                    'Ydata',[e1_w(2,1),e5_w(2,1),e6_w(2,1),e2_w(2,1)],...
                    'Zdata',[e1_w(3,1),e5_w(3,1),e6_w(3,1),e2_w(3,1)]);
    set(robot_fill3,'Xdata',[e1_w(1,1),e5_w(1,1),e7_w(1,1),e3_w(1,1)],...
                    'Ydata',[e1_w(2,1),e5_w(2,1),e7_w(2,1),e3_w(2,1)],...
                    'Zdata',[e1_w(3,1),e5_w(3,1),e7_w(3,1),e3_w(3,1)]);
    set(robot_fill4,'Xdata',[e3_w(1,1),e7_w(1,1),e8_w(1,1),e4_w(1,1)],...
                    'Ydata',[e3_w(2,1),e7_w(2,1),e8_w(2,1),e4_w(2,1)],...
                    'Zdata',[e3_w(3,1),e7_w(3,1),e8_w(3,1),e4_w(3,1)]);
    set(robot_fill5,'Xdata',[e4_w(1,1),e8_w(1,1),e6_w(1,1),e2_w(1,1)],...
                    'Ydata',[e4_w(2,1),e8_w(2,1),e6_w(2,1),e2_w(2,1)],...
                    'Zdata',[e4_w(3,1),e8_w(3,1),e6_w(3,1),e2_w(3,1)]);
    set(robot_fill6,'Xdata',[e4_w(1,1),e3_w(1,1),e1_w(1,1),e2_w(1,1)],...
                    'Ydata',[e4_w(2,1),e3_w(2,1),e1_w(2,1),e2_w(2,1)],...
                    'Zdata',[e4_w(3,1),e3_w(3,1),e1_w(3,1),e2_w(3,1)]);
    
    bp_coor(3,:) = bp_z(:,i);
    bp_coor_w = Trans_bw*bp_coor;
    cable_length = [bp_coor_w(:,1),e1_w,bp_coor_w(:,2),e2_w,bp_coor_w(:,3),e3_w,bp_coor_w(:,4),e4_w];
    set(cl1,'Xdata',cable_length(1,1:2),'Ydata',cable_length(2,1:2),'Zdata',cable_length(3,1:2));
    set(cl2,'Xdata',cable_length(1,3:4),'Ydata',cable_length(2,3:4),'Zdata',cable_length(3,3:4)); 
    set(cl3,'Xdata',cable_length(1,5:6),'Ydata',cable_length(2,5:6),'Zdata',cable_length(3,5:6));
    set(cl4,'Xdata',cable_length(1,7:8),'Ydata',cable_length(2,7:8),'Zdata',cable_length(3,7:8));
    set(cable_start_point,'Xdata',bp_coor_w(1,:),'Ydata',bp_coor_w(2,:),'Zdata',bp_coor_w(3,:));
%     hp = plot3(trj_x(i0:i0+1),trj_y(i0:i0+1),trj_z(i0:i0+1),'Color','b','LineStyle','-','.','MarkerSize',6,...
%     'MarkerEdgeColor','r','MarkerFaceColor','r');

    addpoints(hp,trj_x(i),trj_y(i),trj_z(i)); 
    drawnow;

    if i<length(trj_x)
        pause(t_vec(i+1)-t_vec(i));
    else
        pause(t_vec(i)-t_vec(i-1));
    end
end