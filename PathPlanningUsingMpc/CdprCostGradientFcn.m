function [Gx,Gmv] = CdprCostGradientFcn(stage,x,u,p)
% Analytical gradient of the cost function of the path planner of CDPR


% Wmv = [1 0; 0 1];
% Gx = zeros(3,1);
% Gmv = 2*Wmv*u;

Qp = diag([0 0 0 0.1 0.1]);
Rp =  2 * eye(2);

% Running cost Jacobian
Gx = 2 * Qp *x ;
Gmv = 2 * Rp * u ;
