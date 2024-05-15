function [z0, XY0] = CdprInitialGuess(enable_obs,initialPose,targetPose,u0,p)
% Generate initial guess for decision variables used by the path planner of
% a CDPR system.

% no obstacles
if ~enable_obs
    p1 = round(p/4);
    p2 = round(p/4);
    p3 = round(p/4);
    p4 = p - p1 - p2 - p3 + 1;
    x1 = -2;
    y1 = 0;
    theta1 = pi/12;
    x2 = 0;
    y2 = targetPose(2)/2;
    theta2 = pi/6;
    x3 = 2;
    y3 = targetPose(2);
    theta3 = pi/12;
    xGuess = [linspace(initialPose(1),x1,p1),linspace(x1,x2,p2),linspace(x2,x3,p3),linspace(x3,targetPose(1),p4)];
    yGuess = [linspace(initialPose(2),y1,p1),linspace(y1,y2,p2),linspace(y2,y3,p3),linspace(y3,targetPose(2),p4)];
    thetaGuess = [linspace(initialPose(3),theta1,p1),linspace(theta1,theta2,p2),linspace(theta2,theta3,p3),linspace(theta3,targetPose(3),p4)];
% obstacles exist
else
    p1 = round(p/4);
    p2 = round(p/4);
    p3 = round(p/4);
    p4 = p - p1 - p2 - p3 + 1;
    x1 = -1.5;
    y1 = -2;
    theta1 = pi/6;
    x2 = 0;
    y2 = 2;
    theta2 = 0;
    x3 = 1.5;
    y3 = -2;
    theta3 = -pi/6;
    xGuess = [linspace(initialPose(1),x1,p1),linspace(x1,x2,p2),linspace(x2,x3,p3),linspace(x3,targetPose(1),p4)];
    yGuess = [linspace(initialPose(2),y1,p1),linspace(y1,y2,p2),linspace(y2,y3,p3),linspace(y3,targetPose(2),p4)];
    thetaGuess = [linspace(initialPose(3),theta1,p1),linspace(theta1,theta2,p2),linspace(theta2,theta3,p3),linspace(theta3,targetPose(3),p4)];
end
stateGuess = [xGuess;yGuess;thetaGuess];
z0 = [];
for ct=1:p
    z0 = [z0;stateGuess(:,ct);u0];
end
z0 = [z0;stateGuess(:,ct)];
XY0 = stateGuess(1:2,:)';

