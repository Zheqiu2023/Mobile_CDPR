clear, close all
angle(:, 1) = [-0.054055;-0.858469;-0.018205;-0.509683];
pos(:, 1) = [14.286902;-426.527679;308.638306];

angle(:, 2) = [-0.059072;-0.859948;-0.01792;-0.506635];
pos(:, 2) = [14.23119;-425.719543;308.651001];

angle(:, 3) = [-0.058878;-0.859135;-0.021206;-0.507908];
pos(:, 3) = [14.158152;-425.955536;308.933319];

angle(:, 4) = [-0.056692;-0.86059;-0.013594;-0.505951];
pos(:, 4) = [14.066936;-425.321075;308.97937];

angle(:, 5) = [-0.056476;-0.860405;-0.016468;-0.506204];
pos(:, 5) = [14.334214;-424.999481;308.880249];

max_x_pos = max(pos(1, :)); min_x_pos = min(pos(1, :));
max_y_pos = max(pos(2, :)); min_y_pos = min(pos(2, :));
max_z_pos = max(pos(3, :)); min_z_pos = min(pos(3, :));
max_x_pos_err = max_x_pos - min_x_pos;
max_y_pos_err = max_y_pos - min_y_pos;
max_z_pos_err = max_z_pos - min_z_pos;

% 将每个四元数转换为欧拉角
for i = 1:5
    q = angle(:, i)';
    eulerAngles(i, :) = quat2eul(q);
end
max_alpha = max(eulerAngles(:, 1)); min_alpha = min(eulerAngles(:, 1));
max_beta = max(eulerAngles(:, 2)); min_beta = min(eulerAngles(:, 2));
max_gamma = max(eulerAngles(:, 3)); min_gamma = min(eulerAngles(:, 3));
max_alpha_err = (max_alpha - min_alpha)*180/pi;
max_beta_err = (max_beta - min_beta)*180/pi;
max_gamma_err = (max_gamma - min_gamma)*180/pi;

for i=1:5
    plot3(pos(1, i), pos(2, i), pos(3, i), 'k.', 'MarkerSize', 15); % 黑色圆点表示原点
    hold on;
    R = rotx(eulerAngles(i, 1)) * roty(eulerAngles(i, 2)) * rotz(eulerAngles(i, 3));
    % 绘制旋转后的坐标轴
    i_rotated = R * [1; 0; 0];
    j_rotated = R * [0; 1; 0];
    k_rotated = R * [0; 0; 1];
    arrow_length_factor = 0.2;
    quiver3(pos(1, i), pos(2, i), pos(3, i), i_rotated(1), i_rotated(2), i_rotated(3), arrow_length_factor, 'r'); % 红色表示 X 轴
    quiver3(pos(1, i), pos(2, i), pos(3, i), j_rotated(1), j_rotated(2), j_rotated(3), arrow_length_factor, 'g'); % 绿色表示 Y 轴
    quiver3(pos(1, i), pos(2, i), pos(3, i), k_rotated(1), k_rotated(2), k_rotated(3), arrow_length_factor, 'b'); % 蓝色表示 Z 轴

    % 标注每个点的位置
    str = sprintf('  (%0.1f, %0.1f, %0.1f)', pos(1, i), pos(2, i), pos(3, i)); % 格式化点的位置
    text(pos(1, i), pos(2, i), pos(3, i), str, 'FontSize', 12);
end

legend('','x轴','y轴','z轴');
xlabel('X 轴(mm)');
ylabel('Y 轴(mm)');
zlabel('Z 轴(mm)');
view(3); % 3D 视图
title('重复定位点分布图');



	

