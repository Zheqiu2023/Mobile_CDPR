function cineq = CdprIneqConFcn(stage,x,u,p)
% Inequality constraint function of the path planner of a CDPR system, used
% to avoid static obstacles.

%% Ego and obstacles
persistent obstacles cdpr
L = p(1);
W = p(2);
safetyDistance = 0.5;
if isempty(obstacles)
    %% CDPR
    cdpr = collisionBox(L,W,0);
    %% Obstacles
    obsLength = 0.5; % obstacle length
    obsWidth = 0.5; % obstacle width
    obs1 = collisionBox(obsLength,obsWidth,0);
    T1 = trvec2tform([0, 0, 0]);
    obs1.Pose = T1;
    obstacles = {obs1};
end

%% constraints
% Update cdpr positions
theta = x(3);
xCdpr = x(1) + L/2*cos(theta);
yCdpr = x(2) + L/2*sin(theta);
T = trvec2tform([xCdpr,yCdpr,0]);
H = axang2tform([0 0 1 theta]);
cdpr.Pose = T*H;
% Calculate distances from CDPR to obstacles
numObstacles = numel(obstacles);
distances = zeros(numObstacles,1);
for ct = 1:numObstacles
    distances(ct) = localCheckCollision(cdpr,obstacles{ct});
end
allDistances = distances;
cineq = -allDistances + safetyDistance;

function separationDist = localCheckCollision(geom1, geom2)
%#codegen
needMoreInfo = 1;
if(~coder.target('MATLAB'))
    [collisionStatus, separationDist] = ...
        robotics.core.internal.coder.CollisionGeometryBuildable.checkCollision(...
        geom1.GeometryInternal, geom1.Position, geom1.Quaternion,...
        geom2.GeometryInternal, geom2.Position, geom2.Quaternion,...
        needMoreInfo);
else
    [collisionStatus, separationDist] = ...
        robotics.core.internal.intersect(...
        geom1.GeometryInternal, geom1.Position, geom1.Quaternion,...
        geom2.GeometryInternal, geom2.Position, geom2.Quaternion,...
        needMoreInfo);
end
if collisionStatus
    separationDist = -10;
end


