function trans = trans(pose)
%TRANS RPY角齐次变换矩阵
%   此处显示详细说明
trans = transl(pose(1:3))*trotz(pose(6))*troty(pose(5))*trotx(pose(4));
end

