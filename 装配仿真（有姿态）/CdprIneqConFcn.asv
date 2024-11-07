function cineq = CdprIneqConFcn(stage,x,u,p)
% Inequality constraint function of the path planner, used to avoid static obstacles.

%% Ego and obstacles
persistent obstacles box
L = p(1);
W = p(2);
H = p(3);
safetyDistance = 0.01;
if isempty(obstacles)
    %% CDPR
    box = collisionBox(L,W,H);
    %% Obstacles
    obs1 = collisionBox(0.05,0.1,0.05);
    obs2 = collisionBox(0.1,0.05,0.05);
    obs3 = collisionBox(0.05,0.1,0.05);
    obs4 = collisionBox(0.1,0.05,0.05);
    T1 = trvec2tform([0.051, 0, 0]);
    T2 = trvec2tform([0, 0.051, 0]);
    T3 = trvec2tform([-0.051, 0, 0]);
    T4 = trvec2tform([0, -0.051, 0]);

    obs1.Pose = T1;
    obs2.Pose = T2;
    obs3.Pose = T3;
    obs4.Pose = T4;
    obstacles = {obs1, obs2, obs3, obs4};
end

%% constraints
% Update cdpr positions
T = trvec2tform([x(1),x(2),x(3)]);
matX = axang2tform([1 0 0 x(4)]);
matY = axang2tform([0 1 0 x(5)]);
matZ = axang2tform([0 0 1 x(6)]);
box.Pose = T*matX*matY*matZ;
% Calculate distances from CDPR to obstacles
numObstacles = numel(obstacles);
distances = zeros(numObstacles,1);
for ct = 1:numObstacles
    distances(ct) = localCheckCollision(box,obstacles{ct});
end
allDistances = distances;
cineq = -allDistances;

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


