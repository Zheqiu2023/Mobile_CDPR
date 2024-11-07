function [z0, XYZ0] = CdprInitialGuess(initialPose,targetPose,u0,p)
% Generate initial guess for decision variables used by the path planner of
% a CDPR system.

xGuess = [linspace(initialPose(1),targetPose(1),p)];
yGuess = [linspace(initialPose(2),targetPose(2),p)];
zGuess = [linspace(initialPose(3),targetPose(3),p)];
alphaGuess = [linspace(initialPose(4),targetPose(4),p)];
betaGuess = [linspace(initialPose(5),targetPose(5),p)];
gammaGuess = [linspace(initialPose(6),targetPose(6),p)];
vxGuess = [linspace(initialPose(7),targetPose(7),p)];
vyGuess = [linspace(initialPose(8),targetPose(8),p)];
vzGuess = [linspace(initialPose(9),targetPose(9),p)];
walphaGuess = [linspace(initialPose(10),targetPose(10),p)];
wbetaGuess = [linspace(initialPose(11),targetPose(11),p)];
wgammaGuess = [linspace(initialPose(12),targetPose(12),p)];

stateGuess = [xGuess;yGuess;zGuess;alphaGuess;betaGuess;gammaGuess;vxGuess;vyGuess;vzGuess;walphaGuess;wbetaGuess;wgammaGuess];
z0 = [];
for ct=1:p
    z0 = [z0;stateGuess(:,ct);u0];
end
z0 = [z0;stateGuess(:,ct)];
XYZ0 = stateGuess(1:6,:)';

