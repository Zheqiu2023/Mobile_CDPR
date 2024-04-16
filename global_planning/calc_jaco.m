function [jaco_trans,cable_length] = calc_jaco(param,pose_ee,pose_cdpr)
%% 对于给定末端的执行器的位置 计算雅克比矩阵
% 末端执行器上绳索固定点的坐标    (末端执行器坐标系下)
a1_e = param.ep_coor(:,1);
a2_e = param.ep_coor(:,2);
a3_e = param.ep_coor(:,3);
a4_e = param.ep_coor(:,4);
ao_e = param.ep_o;
% 末端执行器坐标系到世界坐标系的姿态变换矩阵 
Trans_ew = transl(pose_ee(1),pose_ee(2),pose_ee(3)) * trotz(pose_ee(6)) * troty(pose_ee(5)) * trotx(pose_ee(4));
% 世界坐标系下绳索固定点的坐标
a1_w = Trans_ew*a1_e; 
a2_w = Trans_ew*a2_e;
a3_w = Trans_ew*a3_e;
a4_w = Trans_ew*a4_e;
ao_w = Trans_ew*ao_e;

% 绳索起始点的坐标  (基座标系下)
b1_b = param.bp_coor(:,1);
b2_b = param.bp_coor(:,2);
b3_b = param.bp_coor(:,3);
b4_b = param.bp_coor(:,4);
% 基坐标系到世界坐标系的姿态变换矩阵 
Trans_bw = transl(pose_cdpr(1),pose_cdpr(2),0) * trotz(pose_cdpr(3));
% 世界坐标系下绳索起始点的坐标
b1_w = Trans_bw*b1_b;
b2_w = Trans_bw*b2_b;
b3_w = Trans_bw*b3_b;
b4_w = Trans_bw*b4_b;

% 质心到绳索固定点的矢量
vector_ee1 = a1_w(1:3)-ao_w(1:3); 
vector_ee2 = a2_w(1:3)-ao_w(1:3);
vector_ee3 = a3_w(1:3)-ao_w(1:3);
vector_ee4 = a4_w(1:3)-ao_w(1:3);

% 从末端执行器到基座的矢量
vector_cable1 = b1_w(1:3)-a1_w(1:3); 
vector_cable2 = b2_w(1:3)-a2_w(1:3);
vector_cable3 = b3_w(1:3)-a3_w(1:3);
vector_cable4 = b4_w(1:3)-a4_w(1:3);

ideal_length_cable1 = calc_cable_length( a1_w(1:3),b1_w(1:3),param.pulley_radius,param.rotation_radius,param.bp_z_max(1) );
ideal_length_cable2 = calc_cable_length( a2_w(1:3),b2_w(1:3),param.pulley_radius,param.rotation_radius,param.bp_z_max(2) );
ideal_length_cable3 = calc_cable_length( a3_w(1:3),b3_w(1:3),param.pulley_radius,param.rotation_radius,param.bp_z_max(3) );
ideal_length_cable4 = calc_cable_length( a4_w(1:3),b4_w(1:3),param.pulley_radius,param.rotation_radius,param.bp_z_max(4) );
cable_length = [ideal_length_cable1;ideal_length_cable2;ideal_length_cable3;ideal_length_cable4];

% unit cable vector
unit_vector_cable1 = vector_cable1/ideal_length_cable1;
unit_vector_cable2 = vector_cable2/ideal_length_cable2;
unit_vector_cable3 = vector_cable3/ideal_length_cable3;
unit_vector_cable4 = vector_cable4/ideal_length_cable4;

% 力雅可比
jaco_trans = [unit_vector_cable1 unit_vector_cable2...
              unit_vector_cable3 unit_vector_cable4;...
              cross(vector_ee1,unit_vector_cable1)...
              cross(vector_ee2,unit_vector_cable2)...
              cross(vector_ee3,unit_vector_cable3)...
              cross(vector_ee4,unit_vector_cable4)];
% 速度雅可比
jaco = jaco_trans';

% if the determinant of jacobian matrix is 0,then the position is singular       
if (rank(jaco_trans)<4)
%    error('singular position')
end