function [Input_c, State_c] = Next_Point_Planning(start_state, start_input, ep_ref_traj, k, dt, params)
% start_state 起点状态量，只有位置 ref_traj 末端平台参考轨迹，包括位姿和速度 k 预测区间长度 dt 时间步长

import casadi.*;

%待优化状态量
x_state=[];
%待优化量初值
x_state_initial=[];
%约束设置
g_constraint=[];
%约束的下限
g_constraint_lb=[];
%约束的上限
g_constraint_ub=[];

EE = 0;
RR = 0;
Q=eye(12);
R=eye(6);

% 要求解的当前的状态量
num_x = 18; num_u = 8;
x_k=SX.sym('x_c',num_x,1);  % [x_p;y_p;z_p;alpha_p;beta_p;gamma_p;  v_px;v_py;v_pz;w_alpha;w_beta;w_gamma;    x_car;y_car;θ_car;  v_car;alpha_car;w_car]
x_state=[x_state;x_k];
x_state_initial=[x_state_initial;start_state];
% 初始状态量约束
g_constraint_lb=[g_constraint_lb;start_state];
g_constraint_ub=[g_constraint_ub;start_state];
g_constraint=[g_constraint;x_k];

for i=1:k
    % 输入量
    u_k=SX.sym(['u', num2str(i-1)],num_u,1); % [a_px;a_py;a_pz;α_alpha;α_beta;α_gamma;  a_car;α_car]
    v_error = u_k(1:6)-ep_ref_traj(13:18,i+1);
    RR = RR + v_error'*R*v_error;

    x_state=[x_state;u_k];
    x_state_initial=[x_state_initial;start_input];
    % 输入量约束
    g_constraint_lb=[g_constraint_lb;[-2;-2;-2;-0.3;-0.3;-0.3;  -0.05;-0.05]];
    g_constraint_ub=[g_constraint_ub;[2;2;2;0.3;0.3;0.3;    0.05;0.05]];
    g_constraint=[g_constraint;u_k];

    % 状态方程
    x_n = [x_k(1) + dt*x_k(7) + dt^2*u_k(1)/2;
        x_k(2) + dt*x_k(8) + dt^2*u_k(2)/2;
        x_k(3) + dt*x_k(9) + dt^2*u_k(3)/2;
        x_k(4) + dt*x_k(10) + dt^2*u_k(4)/2;
        x_k(5) + dt*x_k(11) + dt^2*u_k(5)/2;
        x_k(6) + dt*x_k(12) + dt^2*u_k(6)/2;

        x_k(7) + dt*u_k(1);
        x_k(8) + dt*u_k(2);
        x_k(9) + dt*u_k(3);
        x_k(10) + dt*u_k(4);
        x_k(11) + dt*u_k(5);
        x_k(12) + dt*u_k(6);

        x_k(13) + dt*x_k(16)*cos(x_k(15));
        x_k(14) + dt*x_k(16)*sin(x_k(15));
        x_k(15) + dt*x_k(16)*tan(x_k(17))/params.L;

        x_k(16) + dt*u_k(7);
        x_k(17) + dt*x_k(18);
        x_k(18) + dt*u_k(8);];

    % 第K个预测状态
    x_k=SX.sym(['x_', num2str(i)],num_x,1);
    x_state=[x_state;x_k];
    x_state_initial=[x_state_initial;start_state];
    % 中间状态量约束
    g_constraint_lb=[g_constraint_lb;[-10;-10;0;-pi/18;-pi/18;-pi/2;    -2;-2;-2;-0.3;-0.3;-0.3;    -10;-10;-pi/2;  -2;-pi/6;-0.1]];
    g_constraint_ub=[g_constraint_ub;[10;10;3;pi/18;pi/18;pi/2;   2;2;2;0.3;0.3;0.3;    10;10;pi/2;   2;pi/6;0.1]];
    g_constraint=[g_constraint;x_k];
    % x(k) = A*x(k-1) + B*u
    g_constraint_lb=[g_constraint_lb;zeros(num_x, 1)];
    g_constraint_ub=[g_constraint_ub;zeros(num_x, 1)];
    g_constraint=[g_constraint;x_k-x_n];
    % 限制车身位置与末端平台位置差
    g_constraint_lb=[g_constraint_lb;[0;0]];
    g_constraint_ub=[g_constraint_ub;[0;0]];
    g_constraint=[g_constraint;[x_k(1)-x_k(13)-params.L/2*cos(x_k(15)); x_k(2)-x_k(14)-params.L/2*sin(x_k(15))]];
    % 车身转角与末端平台绕z轴姿态角相等
    g_constraint_lb=[g_constraint_lb;0];
    g_constraint_ub=[g_constraint_ub;0];
    g_constraint=[g_constraint;x_k(6)-x_k(15)];
    % 障碍物约束
    dis = obs_con(x_k(13:15), [], params);
    g_constraint_lb=[g_constraint_lb;0];
    g_constraint_ub=[g_constraint_ub;inf];
    g_constraint=[g_constraint;dis];
    % 速度约束
    %     g_constraint_lb=[g_constraint_lb;-pi/2];
    %     g_constraint_ub=[g_constraint_ub;pi/2];
    %     g_constraint=[g_constraint;atan2(x_k(8),x_k(7))];
    % 轨迹误差
    x_error1=x_k(1:12)-ep_ref_traj(1:12,i+1);
    EE=EE+x_error1'*Q*x_error1;
end

%% 设置目标
Obj= EE;

nlp=struct('x',x_state,'f',Obj,'g',g_constraint);
Solver=nlpsol('Solver','ipopt',nlp);
r=Solver('x0',x_state_initial,'lbg',g_constraint_lb,'ubg',g_constraint_ub);
Trajectory_x=r.x;

State_c=[];
Input_c=[];

p=1;
State_c=[State_c;Trajectory_x(p:p+num_x-1,1)];
p=p+num_x;
for i=1:k
    Input_c=[Input_c;Trajectory_x(p:p+num_u-1,1)];
    p=p+num_u;
    State_c=[State_c;Trajectory_x(p:p+num_x-1,1)];
    p=p+num_x;
end

State_c=full(State_c);
State_c=reshape(State_c,num_x,[]);
Input_c=full(Input_c);
Input_c=reshape(Input_c,num_u,[]);
