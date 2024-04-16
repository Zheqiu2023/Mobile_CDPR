function ideal_cable_length = calc_cable_length( a,b,pulley_radius,rotation_radius,bp_z_max )  % 动平台连接点；锚点座坐标；滑轮半径；旋转半径
theta1 = abs(atan((a(1)-b(1))/(a(2)-b(2))));    % 滑轮面与纸面夹角
Ox = b(1) + sign(a(1)-b(1))*rotation_radius*sin(theta1);      
Oy = b(2) + sign(a(2)-b(2))*rotation_radius*cos(theta1);
Oz = b(3);
O = [Ox Oy Oz]';    % 出绳滑轮中心坐标
l1 = norm(O(1:3)-a(1:3));   % |oa|
l2 = (l1^2-pulley_radius^2)^0.5;

theta3 = atan(l2 / pulley_radius);
theta4 = asin(abs(a(3)-O(3)) / l1);
theta2 = pi/2 - (theta3 - theta4);
c = theta2 * pulley_radius;

l3 = bp_z_max - Oz;

ideal_cable_length = l2 + c + l3;

