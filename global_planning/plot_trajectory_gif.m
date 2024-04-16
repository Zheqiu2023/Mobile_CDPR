function [] = plot_trajectory_gif(t_vec, pose_ee)
trj_x = pose_ee(1,:);
trj_y = pose_ee(2,:);
trj_z = pose_ee(3,:);
%% position
figure(1)
plot3(trj_x(1),trj_y(1),trj_z(1),'b.','Markersize',10)
% axis equal
axis([-1 4 -1 4 0 1.5])
grid on
%view([0 0 1])
hold on
pause(0.5)
for i0=1:length(trj_x)-1
    hp = plot3(trj_x(i0:i0+1),trj_y(i0:i0+1),trj_z(i0:i0+1));
    hp.LineStyle='-';
    hp.Color='b';
    hp.Marker='.';
    hp.MarkerSize=8;
    hp.MarkerEdgeColor='r';
    hp.MarkerFaceColor='r';
    hold on
    pause(t_vec(i0+1)-t_vec(i0))
end
%% speed and acceleration
%{
hold off
% speed
figure(2)
V_x=(trj_x(2:end)-trj_x(1:end-1))./(t_vec(2:end)-t_vec(1:end-1));
V_y=(trj_y(2:end)-trj_y(1:end-1))./(t_vec(2:end)-t_vec(1:end-1));
V_z=(trj_z(2:end)-trj_z(1:end-1))./(t_vec(2:end)-t_vec(1:end-1));
plot(t_vec(2:end),V_x,t_vec(2:end),V_y,t_vec(2:end),V_z) 
legend('Vx','Vy','Vz')
% acceleration
figure (3)
A_x=(V_x(2:end)-V_x(1:end-1))./(t_vec(3:end)-t_vec(2:end-1));
A_y=(V_y(2:end)-V_y(1:end-1))./(t_vec(3:end)-t_vec(2:end-1));
A_z=(V_z(2:end)-V_z(1:end-1))./(t_vec(3:end)-t_vec(2:end-1));
plot(t_vec(3:end),A_x,t_vec(3:end),A_y,t_vec(3:end),A_z) 
legend('Ax','Ay','Az')
%}