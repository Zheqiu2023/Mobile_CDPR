function f=direct_kine_lsqnonlin(x,l,d,param_cdpr)
%% variables instruction
%x=[xx_g yy_g zz_g angle_x angle_y angle_z cf1 cf2 cf3 cf4]
% 合外力
force_ee=[0;0;-param_cdpr.M_ee*param_cdpr.g];
% 合外力矩
moment_ee=[0;0;0];
%% equations deduce
[jaco_trans,cable_length] = calc_jaco(param_cdpr,x,d);
        
% 10 equations(已知绳长和cdpr位姿，10个方程求解10个未知数)
f(1:4)=l-cable_length;
% force equilibrium equations J'*T+F=0
equ_j=jaco_trans*x(7:10)+[force_ee;moment_ee];
f(5:10)=equ_j;