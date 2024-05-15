function [A, B] = CdprStateJacobianFcn(x, u, p)
% Jacobian of model equations for parking.
% state variables x, y and yaw angle theta.
% control variables v and steering angle delta.
%
%   Units: length/position in "m", velocity in "m/s" and angle in "radian".
%
%   All angles are set positive counter-clockwise.
%
%   It returns A and B such that "dxdt = Ax + Bu" is the linearized plant
%   at {x, u}.


theta = x(3);  
alpha = u(1);
v = u(2);
L = p(1);
% Linearize the state equations at the current condition
A = zeros(3,3);
B = zeros(3,2);
A(1,3) = v*(-sin(theta));
A(2,3) = v*cos(theta);
B(3,1) = v/(L*cos(alpha)^2);
B(1,2) = cos(theta);
B(2,2) = sin(theta);
B(3,2) = tan(alpha)/L;
