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
v = x(4);
alpha = x(5);

a = u(1);
w = u(2);

L = p(1);
% Linearize the state equations at the current condition
A = zeros(5,5);
B = zeros(5,2);

A(1,3) = v*(-sin(theta));
A(1,4) = cos(theta); 

A(2,3) = v*cos(theta);
A(2,4) = sin(theta);

A(3,4) = tan(alpha)/L;
A(3,5) = v/(L*cos(alpha)^2);

B(4,1) = 1;
B(5,2) = 1;
