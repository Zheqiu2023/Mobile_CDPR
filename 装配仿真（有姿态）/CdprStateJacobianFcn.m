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

dt = p(1);

% Linearize the state equations at the current condition
A = zeros(12,12);
B = zeros(12,6);

A(1,7) = 1; 
A(2,8) = 1; 
A(3,9) = 1; 
A(4,10) = 1; 
A(5,11) = 1; 
A(6,12) = 1; 

B(1,1) = dt;
B(2,2) = dt;
B(3,3) = dt;
B(4,4) = dt;
B(5,5) = dt;
B(6,6) = dt;
B(7,1) = 1;
B(8,2) = 1;
B(9,3) = 1;
B(10,4) = 1;
B(11,5) = 1;
B(12,6) = 1;
