function [Gx,Gmv] = CdprCostGradientFcn(stage,x,u,p)
% Analytical gradient of the cost function of the path planner of CDPR

Qp = diag([0 0 0 0 0 0 0.1 0.1 0.1 0.1 0.1 0.1]);
Rp = 6*eye(6);

% Running cost Jacobian
Gx = 2 * Qp *x ;
Gmv = 2 * Rp * u ;
