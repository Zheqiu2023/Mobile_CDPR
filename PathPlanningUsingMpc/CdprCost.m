function cost =  TruckTrailerCost(stage,x,u,p)
% Cost function of the path planner of CDPR

% Wmv = [1 0; 0 1];
% cost = u'*Wmv*u;

Qp = diag([0.1 0.1 0.1]);
Rp = 2*eye(2);

% process cost
cost =  x'*Qp*x + u'*Rp*u;


