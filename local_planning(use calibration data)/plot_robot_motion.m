function [] = plot_robot_motion(t_vec, pose_ee, h, param_cdpr)
% position
trj_x = pose_ee(1,:);
trj_y = pose_ee(2,:);
trj_z = pose_ee(3,:);

%% 获取屏幕大小，Screensize是一个4元素向量[left, bottom, width, height]，然后用获得的screensize向量设置fig的position属性
figure(1)
scrsz = get(0,'ScreenSize');  
set(gcf,'Position',[0 0 scrsz(3) scrsz(4)-90]);    
%% 绘制锚点座运动范围
v1_g=[param_cdpr.bp_coor(1,1);param_cdpr.bp_coor(2,1);param_cdpr.bp_z_max(1);1];   % 与b1_g相同
v2_g=[param_cdpr.bp_coor(1,2);param_cdpr.bp_coor(2,2);param_cdpr.bp_z_max(2);1];
v3_g=[param_cdpr.bp_coor(1,3);param_cdpr.bp_coor(2,3);param_cdpr.bp_z_max(3);1];
v4_g=[param_cdpr.bp_coor(1,4);param_cdpr.bp_coor(2,4);param_cdpr.bp_z_max(4);1];
v5_g=[param_cdpr.bp_coor(1,1);param_cdpr.bp_coor(2,1);param_cdpr.bp_z_min(1);1];
v6_g=[param_cdpr.bp_coor(1,2);param_cdpr.bp_coor(2,2);param_cdpr.bp_z_min(2);1];
v7_g=[param_cdpr.bp_coor(1,3);param_cdpr.bp_coor(2,3);param_cdpr.bp_z_min(3);1];
v8_g=[param_cdpr.bp_coor(1,4);param_cdpr.bp_coor(2,4);param_cdpr.bp_z_min(4);1];
v_g=[v1_g,v2_g,v4_g,v3_g,v1_g,v5_g,v6_g,v2_g,v6_g,v8_g,v4_g,v8_g,v7_g,v3_g,v7_g,v5_g];  % 运动范围向量
plot3(v_g(1,:),v_g(2,:),v_g(3,:),'r','Linewidth',1);
hold on
xlabel('X');ylabel('Y');zlabel('Z');view(30,30);

Tr_matrix=transl(pose_ee(1),pose_ee(2),pose_ee(3))*trotz(pose_ee(6))*troty(pose_ee(5))*trotx(pose_ee(4));
% 机器人模型为长方体，e1~e8为8个顶点
e1_g=Tr_matrix*param_cdpr.ep_coor(:,1); % 与a1_g相同
e2_g=Tr_matrix*param_cdpr.ep_coor(:,2);
e3_g=Tr_matrix*param_cdpr.ep_coor(:,3);
e4_g=Tr_matrix*param_cdpr.ep_coor(:,4);
e5_g=Tr_matrix*[param_cdpr.ep_coor(1,1);param_cdpr.ep_coor(2,1);-param_cdpr.ep_coor(3,1);1];
e6_g=Tr_matrix*[param_cdpr.ep_coor(1,2);param_cdpr.ep_coor(2,2);-param_cdpr.ep_coor(3,2);1];
e7_g=Tr_matrix*[param_cdpr.ep_coor(1,3);param_cdpr.ep_coor(2,3);-param_cdpr.ep_coor(3,3);1];
e8_g=Tr_matrix*[param_cdpr.ep_coor(1,4);param_cdpr.ep_coor(2,4);-param_cdpr.ep_coor(3,4);1];
%% 绘制机器人模型
e_g=[e1_g,e2_g,e4_g,e3_g,e1_g,e5_g,e6_g,e2_g,e6_g,e8_g,e4_g,e8_g,e7_g,e3_g,e7_g,e5_g];  % 机器人框架向量
robot_frame=plot3(e_g(1,:),e_g(2,:),e_g(3,:),'k','Linewidth',1);
robot_fill=fill3([e5_g(1,1),e6_g(1,1),e8_g(1,1),e7_g(1,1)],...
                 [e5_g(2,1),e6_g(2,1),e8_g(2,1),e7_g(2,1)],...
                 [e5_g(3,1),e6_g(3,1),e8_g(3,1),e7_g(3,1)],'g');    % 底面
robot_fill2=fill3([e1_g(1,1),e5_g(1,1),e6_g(1,1),e2_g(1,1)],...
                  [e1_g(2,1),e5_g(2,1),e6_g(2,1),e2_g(2,1)],...
                  [e1_g(3,1),e5_g(3,1),e6_g(3,1),e2_g(3,1)],'y');
robot_fill3=fill3([e1_g(1,1),e5_g(1,1),e7_g(1,1),e3_g(1,1)],...
                  [e1_g(2,1),e5_g(2,1),e7_g(2,1),e3_g(2,1)],...
                  [e1_g(3,1),e5_g(3,1),e7_g(3,1),e3_g(3,1)],'y');
robot_fill4=fill3([e3_g(1,1),e7_g(1,1),e8_g(1,1),e4_g(1,1)],...
                  [e3_g(2,1),e7_g(2,1),e8_g(2,1),e4_g(2,1)],...
                  [e3_g(3,1),e7_g(3,1),e8_g(3,1),e4_g(3,1)],'y');
robot_fill5=fill3([e4_g(1,1),e8_g(1,1),e6_g(1,1),e2_g(1,1)],...
                  [e4_g(2,1),e8_g(2,1),e6_g(2,1),e2_g(2,1)],...
                  [e4_g(3,1),e8_g(3,1),e6_g(3,1),e2_g(3,1)],'y');
robot_fill6=fill3([e4_g(1,1),e3_g(1,1),e1_g(1,1),e2_g(1,1)],...
                  [e4_g(2,1),e3_g(2,1),e1_g(2,1),e2_g(2,1)],...
                  [e4_g(3,1),e3_g(3,1),e1_g(3,1),e2_g(3,1)],'g');   % 顶面
%% 绘制绳索
cable_length=[v1_g,e1_g,v2_g,e2_g,v3_g,e3_g,v4_g,e4_g];
cl1=plot3(cable_length(1,1:2),cable_length(2,1:2),cable_length(3,1:2),'k');
cl2=plot3(cable_length(1,3:4),cable_length(2,3:4),cable_length(3,3:4),'k');
cl3=plot3(cable_length(1,5:6),cable_length(2,5:6),cable_length(3,5:6),'k');
cl4=plot3(cable_length(1,7:8),cable_length(2,7:8),cable_length(3,7:8),'k');
%% 绘制机器人中心点
central_point=plot3(trj_x(1),trj_y(1),trj_z(1),'o','MarkerSize',6,'MarkerEdgeColor','r','MarkerFaceColor','r');
%% 绘制绳索起始点
cable_start_point=scatter3(param_cdpr.bp_coor(1,:),param_cdpr.bp_coor(2,:),param_cdpr.bp_coor(3,:),200,'g','.');

%% 机器人运动动画
% hp = plot3(trj_x(1:2),trj_y(1:2),trj_z(1:2),'Color','b','LineStyle','-','.','MarkerSize',6,...
%     'MarkerEdgeColor','r','MarkerFaceColor','r');
hp=animatedline('Color','b','LineStyle','-');

axis equal
% axis([-0.5 0.5 -0.5 0.5 -0.2 0.6])
grid on
xlim([min(param_cdpr.bp_coor(1,:)),max(param_cdpr.bp_coor(1,:))])
ylim([min(param_cdpr.bp_coor(2,:)),max(param_cdpr.bp_coor(2,:))])
% zlim([0,max(param_cdpr.bp_coor(3,:))+100])
pause(0.5)

for i=1:length(trj_x)
    Tr_matrix=transl(pose_ee(1,i),pose_ee(2,i),pose_ee(3,i))*trotz(pose_ee(6,i))*troty(pose_ee(5,i))*trotx(pose_ee(4,i));
    e1_g=Tr_matrix*param_cdpr.ep_coor(:,1);
    e2_g=Tr_matrix*param_cdpr.ep_coor(:,2);
    e3_g=Tr_matrix*param_cdpr.ep_coor(:,3);
    e4_g=Tr_matrix*param_cdpr.ep_coor(:,4);
    e5_g=Tr_matrix*[param_cdpr.ep_coor(1,1);param_cdpr.ep_coor(2,1);-param_cdpr.ep_coor(3,1);1];
    e6_g=Tr_matrix*[param_cdpr.ep_coor(1,2);param_cdpr.ep_coor(2,2);-param_cdpr.ep_coor(3,2);1];
    e7_g=Tr_matrix*[param_cdpr.ep_coor(1,3);param_cdpr.ep_coor(2,3);-param_cdpr.ep_coor(3,3);1];
    e8_g=Tr_matrix*[param_cdpr.ep_coor(1,4);param_cdpr.ep_coor(2,4);-param_cdpr.ep_coor(3,4);1];
    e_g=[e1_g,e2_g,e4_g,e3_g,e1_g,e5_g,e6_g,e2_g,e6_g,e8_g,e4_g,e8_g,e7_g,e3_g,e7_g,e5_g];
    set(robot_frame,'Xdata',e_g(1,:),'Ydata',e_g(2,:),'Zdata',e_g(3,:));
    set(robot_fill,'Xdata',[e5_g(1,1),e6_g(1,1),e8_g(1,1),e7_g(1,1)],...
                   'Ydata',[e5_g(2,1),e6_g(2,1),e8_g(2,1),e7_g(2,1)],...
                   'Zdata',[e5_g(3,1),e6_g(3,1),e8_g(3,1),e7_g(3,1)]);
    set(robot_fill2,'Xdata',[e1_g(1,1),e5_g(1,1),e6_g(1,1),e2_g(1,1)],...
                    'Ydata',[e1_g(2,1),e5_g(2,1),e6_g(2,1),e2_g(2,1)],...
                    'Zdata',[e1_g(3,1),e5_g(3,1),e6_g(3,1),e2_g(3,1)]);
    set(robot_fill3,'Xdata',[e1_g(1,1),e5_g(1,1),e7_g(1,1),e3_g(1,1)],...
                    'Ydata',[e1_g(2,1),e5_g(2,1),e7_g(2,1),e3_g(2,1)],...
                    'Zdata',[e1_g(3,1),e5_g(3,1),e7_g(3,1),e3_g(3,1)]);
    set(robot_fill4,'Xdata',[e3_g(1,1),e7_g(1,1),e8_g(1,1),e4_g(1,1)],...
                    'Ydata',[e3_g(2,1),e7_g(2,1),e8_g(2,1),e4_g(2,1)],...
                    'Zdata',[e3_g(3,1),e7_g(3,1),e8_g(3,1),e4_g(3,1)]);
    set(robot_fill5,'Xdata',[e4_g(1,1),e8_g(1,1),e6_g(1,1),e2_g(1,1)],...
                    'Ydata',[e4_g(2,1),e8_g(2,1),e6_g(2,1),e2_g(2,1)],...
                    'Zdata',[e4_g(3,1),e8_g(3,1),e6_g(3,1),e2_g(3,1)]);
    set(robot_fill6,'Xdata',[e4_g(1,1),e3_g(1,1),e1_g(1,1),e2_g(1,1)],...
                    'Ydata',[e4_g(2,1),e3_g(2,1),e1_g(2,1),e2_g(2,1)],...
                    'Zdata',[e4_g(3,1),e3_g(3,1),e1_g(3,1),e2_g(3,1)]);
    
    [~, ~, Z] = fit_archor_coor(h(:,i));
    bp_coor = param_cdpr.bp_coor;
    bp_coor(3, :) = Z;
    
    cable_length = [bp_coor(:,1),e1_g,bp_coor(:,2),e2_g,bp_coor(:,3),e3_g,bp_coor(:,4),e4_g];
    set(cl1,'Xdata',cable_length(1,1:2),'Ydata',cable_length(2,1:2),'Zdata',cable_length(3,1:2));
    set(cl2,'Xdata',cable_length(1,3:4),'Ydata',cable_length(2,3:4),'Zdata',cable_length(3,3:4)); 
    set(cl3,'Xdata',cable_length(1,5:6),'Ydata',cable_length(2,5:6),'Zdata',cable_length(3,5:6));
    set(cl4,'Xdata',cable_length(1,7:8),'Ydata',cable_length(2,7:8),'Zdata',cable_length(3,7:8));
    
    set(cable_start_point,'Xdata',bp_coor(1,:),'Ydata',bp_coor(2,:),'Zdata',bp_coor(3,:)); 
    set(central_point,'Xdata',trj_x(i),'Ydata',trj_y(i),'Zdata',trj_z(i));
    
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