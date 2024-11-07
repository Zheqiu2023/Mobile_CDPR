function cost =  CdprCost(stage,x,u,p)
% Cost function of the path planner of CDPR

Qp = diag([0 0 0 0 0 0 0.1 0.1 0.1 0.1 0.1 0.1]);
Rp = 6*eye(6);

% process cost
cost =  x'*Qp*x + u'*Rp*u;


