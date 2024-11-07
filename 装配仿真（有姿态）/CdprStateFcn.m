function dxdt = CdprStateFcn(x,u,p)
%   Units: length/position in "m", velocity in "m/s" and angle in "radian".
%
%   All angles are set positive counter-clockwise.

v_x = x(7);
v_y = x(8);
v_z = x(9);
w_alpha = x(10);
w_beta = x(11);
w_gamma = x(12);

a_x = u(1);
a_y = u(2);
a_z = u(3);
a_alpha = u(4);
a_beta = u(5);
a_gamma = u(6);

dt = p(1);

dxdt = zeros(12,1);
dxdt(1) = v_x+a_x*dt;
dxdt(2) = v_y+a_y*dt;
dxdt(3) = v_z+a_z*dt;
dxdt(4) = w_alpha+a_alpha*dt;
dxdt(5) = w_beta+a_beta*dt;
dxdt(6) = w_gamma+a_gamma*dt;
dxdt(7) = a_x;
dxdt(8) = a_y;
dxdt(9) = a_z;
dxdt(10) = a_alpha;
dxdt(11) = a_beta;
dxdt(12) = a_gamma;
