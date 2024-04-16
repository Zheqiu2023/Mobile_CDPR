function f=direct_kine_lsqnonlin(x,l,param)
%x=[xx_g yy_g zz_g angle_x angle_y angle_z cf1 cf2 cf3 cf4]
% 合外力
force_ee=[0;0;-param.M_ee*param.g];
% 合外力矩
moment_ee=[0;0;0];
%% equations deduce
[jaco_trans,cable_length] = calc_jaco(param,x(1:6));
        
% force equilibrium equations J'*T+F=0
equ_j=jaco_trans*x(7:10) + [force_ee;moment_ee];
% 10 equations(已知绳长，10个方程求解10个未知数)
f(1:4) = l - cable_length;
f(5:10) = equ_j;