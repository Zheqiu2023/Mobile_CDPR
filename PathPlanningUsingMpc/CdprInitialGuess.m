function [z0, XY0] = CdprInitialGuess(enable_obs,initialPose,targetPose,u0,p)
% Generate initial guess for decision variables used by the path planner of
% a CDPR system.

xGuess = [linspace(initialPose(1),targetPose(1),p)];
yGuess = [linspace(initialPose(2),targetPose(2),p)];
thetaGuess = [linspace(initialPose(3),targetPose(3),p)];
vGuess = [linspace(initialPose(4),targetPose(4),p)];
alphaGuess = [linspace(initialPose(5),targetPose(5),p)];

stateGuess = [xGuess;yGuess;thetaGuess;vGuess;alphaGuess];
z0 = [];
for ct=1:p
    z0 = [z0;stateGuess(:,ct);u0];
end
z0 = [z0;stateGuess(:,ct)];
XY0 = stateGuess(1:2,:)';

