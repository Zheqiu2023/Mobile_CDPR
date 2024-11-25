function [dis1,dis2,dis3,dis4,dis5] = obs_con(car_pose, ep_pose, params)
% Inequality constraint function of the path planner of a CDPR system, used
% to avoid static obstacles.

%% Ego and obstacles
L = params.L;
W = params.W;
R_car = L/2;
R_leg = 0.15;
R_ep = 0.05;
R_obs = 0.15;
H_obs = 0.15;
obs_pos = [0 0 H_obs/2];

safetyDistance = 0.1;


%% constraints
% Update cdpr positions
theta = car_pose(3);
x = car_pose(1);
y = car_pose(2);

x_leg_lf = x + L*cos(theta) - W*sin(theta)/2;
y_leg_lf = y + L*sin(theta) + W*cos(theta)/2;

x_leg_rf = x + L*cos(theta) + W*sin(theta)/2;
y_leg_rf = y + L*sin(theta) - W*cos(theta)/2;

x_leg_lb = x - W*sin(theta)/2;
y_leg_lb = y + W*cos(theta)/2;

x_leg_rb = x + W*sin(theta)/2;
y_leg_rb = y - W*cos(theta)/2;

% 车身与障碍物间距
dis1 = sqrt((x_leg_lf-obs_pos(1))^2 + (y_leg_lf-obs_pos(2))^2) - R_leg - R_obs;
dis2 = sqrt((x_leg_rf-obs_pos(1))^2 + (y_leg_rf-obs_pos(2))^2) - R_leg - R_obs;
dis3 = sqrt((x_leg_lb-obs_pos(1))^2 + (y_leg_lb-obs_pos(2))^2) - R_leg - R_obs;
dis4 = sqrt((x_leg_rb-obs_pos(1))^2 + (y_leg_rb-obs_pos(2))^2) - R_leg - R_obs;
% 末端平台与障碍物间距
dis5 = sqrt((ep_pose(1)-obs_pos(1))^2 + (ep_pose(2)-obs_pos(2))^2 + (ep_pose(3)-obs_pos(3))^2) - R_ep - R_obs;




