function [L, unit_vec, vec_anchorPM, pulCenter, pulRotAngle] = CDPRInvKine(P, anchor_P, anchor_M, z_max, pulley_radius, rotation_radius)
%CDPRINVKINE 计算CDPR逆运动学，算出绳长，出绳滑轮转角，绳长向量
%    input describtion:
%    P: pose and position of the platform; 6X1
%    anchor_P: coordinate of the platform anchor in P-coordinate; 6X1
%    anchor_M: coordinate of the moving anchor in G-coordinate; 6X1
%    z_max: max z coordinate of the moving anchor
%    pulley_radius: radius of pully; 
%    rotation_radius: rotation radius of the moving anchor;
%    rotz_angle: moving anchor Z-rotation angle relative to G-coordinate
              
%基本参数
anchor_M = anchor_M(1:3);   % 动锚点座坐标（全局坐标系下）
anchor_P = trans(P)*([anchor_P(1:3);1]);    % 将动平台锚点坐标转换到全局坐标系
anchor_P = anchor_P(1:3);   % 动平台锚点坐标（全局坐标系下）
vec_anchorPM = anchor_M - anchor_P; 
unit_vec = vec_anchorPM/norm(vec_anchorPM);

pulRotAngle = abs(atan(vec_anchorPM(1)/vec_anchorPM(2)));    % 滑轮面与纸面夹角
Ox = anchor_M(1) + sign(-vec_anchorPM(1))*rotation_radius*sin(pulRotAngle);      
Oy = anchor_M(2) + sign(-vec_anchorPM(2))*rotation_radius*cos(pulRotAngle);
Oz = anchor_M(3);
pulCenter = [Ox Oy Oz]';    % 出绳滑轮中心坐标
l1 = norm(pulCenter(1:3)-anchor_P(1:3));
l2 = (l1^2-pulley_radius^2)^0.5;

theta3 = atan(l2 / pulley_radius);
theta4 = asin(abs(anchor_P(3)-pulCenter(3)) / l1);
theta2 = pi/2 - (theta3 - theta4);  % 绳索在滑轮上绕过的弧长对应圆心角
l3 = theta2 * pulley_radius;

l4 = z_max - Oz;

L = l2 + l3 +l4;    % 绳长
