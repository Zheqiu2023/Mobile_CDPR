function dxdt = CdprStateFcn(x,u,p)
% CDPR dynamic system:
%
%   States:
%       1: x (center of the CDPR's rear axle, global x position)
%       2: y (center of the CDPR's rear axle, global y position)
%       3: theta (CDPR orientation, global angle, 0 = east)
%
%   Inputs:
%       1: alpha (CDPR steering angle)
%       2: v (CDPR longitudinal velocity)
%
%   Parameters:
%       p(1): L (CDPR length)
%
%   Units: length/position in "m", velocity in "m/s" and angle in "radian".
%
%   All angles are set positive counter-clockwise.

theta = x(3);
v = x(4);
alpha = x(5);

a = u(1);
w = u(2);

L = p(1);
dxdt = zeros(3,1);
dxdt(1) = v*cos(theta);
dxdt(2) = v*sin(theta);
dxdt(3) = v*tan(alpha)/L;
dxdt(4) = a;
dxdt(5) = w;