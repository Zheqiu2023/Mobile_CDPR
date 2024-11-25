function dis = obs_con(car_pose, ep_pose, params)
% Inequality constraint function of the path planner of a CDPR system, used
% to avoid static obstacles.

%% Ego and obstacles
L = params.L;
W = params.W;
R_car = L/2;
R_obs = 0.25;
obs_pos = [0 0 R_obs];

safetyDistance = 0.1;


%% constraints
% Update cdpr positions
theta = car_pose(3);
xCdpr = car_pose(1) + L/2*cos(theta);
yCdpr = car_pose(2) + L/2*sin(theta);

dis = sqrt((xCdpr-obs_pos(1))^2 + (yCdpr-obs_pos(2))^2) - R_car - R_obs - safetyDistance;




