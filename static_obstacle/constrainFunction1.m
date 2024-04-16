function [C, Ceq]=constrainFunction1(X, Platform_Next, PlatAnchorPosition_Now, MovAnchor, Cable, ...
                                     obsBB, obsNumber, vertex, face, pulley_radius, rotation_radius, wrenchSetting)
%% 计算约束函数constrainFunction1（优化算法, 变量：步长+拉力）
%说明：计算绳索间距，绳索与障碍物间距，绳索与动平台间距的不等式约束（小于等于）
%   以及当绳索间距小于一定距离时的干涉情况（用0和1表示的等式约束）
%输入：下一点的优化参量X（包含步长x_step，绳拉力F），上一点的CdprPlatform对象Platform_Next，
%     当前动平台锚点座位置PlatAnchorPosition，当前CdprMovAnchor对象MovAnchor，当前CdprCable对象Cable，ObsBB边界框对象obsBB，
%     边界框数量obsNumber，动平台包络多面体的几何信息vertex和face，滑轮半径pulley_radius， 平台力分布参数设置对象wrenchSetting（WrenchSetting类）
%输出：不等式约束C，等式约束Ceq
%% 初始化并记录当前点的绳长向量、绳间距离
x_step = X(1:4); %4个锚点座的步长
tension = X(9:16); %绳拉力
TanLinVector_Now = Cable.TanLinVector;
ccDistance_Now = Cable.ccDistance;
%% 更新动锚点座位置
MovAnchor=MovAnchor.setPosL(MovAnchor.positon_L + x_step);
%% 计算绳长向量
for i=1:Cable.cableNumber
    [Cable.Length(i), Cable.UniVector(:,i), Cable.TanLinVector(:,i), ...
                    MovAnchor.PulCenter(:,i), MovAnchor.PulRotz_L(i)] ...
    = CDPRInvKine(Platform_Next.pose_G, Platform_Next.anchorPosition_P(:,i), ...
                    MovAnchor.positon_G(:,i), MovAnchor.endPosition_L(i),pulley_radius, rotation_radius);
end

%% 计算绳索到障碍物的距离(openGJK)(正常值)
for i=1:obsNumber
    for j=1:8
        cable = [Cable.TanLinVector(:, j) + Platform_Next.anchorPosition_G(:, j), Platform_Next.anchorPosition_G(:, j)];
        Cable.coDistance(i, j) = openGJK(cable, obsBB(i).vertices); %openGJK
    end
end

%% 计算绳索与动平台之间的距离（正弦平方值）
[Cable.cpDistance, Cable.voronoi]=updateVoronoi(Cable.UniVector, vertex, face);

%% 计算绳索之间的距离（平方值）
Cable.ccDistance=updateCableDis(Platform_Next, Cable, vertex);

%% 计算动平台到障碍物距离(openGJK)(正常值)
% for i=1:obsNumber
%     Platform_Next.poDistance(i) = openGJK(Platform_Next.anchorPosition_G, obsBB(i).vertices);
% end

%% 等式约束
Ceq = zeros(1, 28 + 6); %等式约束：28个绳间干涉检查+6个力平衡分量
%绳穿越干涉检查(当前点绳间距离小于一定值时检查)
%遍历两两绳之间
sqr_ccDis_Intersect = Cable.ccDis_Intersect * Cable.ccDis_Intersect;
for i=1:Cable.cableNumber
	for j=(i+1):Cable.cableNumber
		index = (j-i) + (7+(9-i))*(i-1)/2;
        if ccDistance_Now(index) < sqr_ccDis_Intersect %绳间距离小于最短检测距离时
            test1 = [(TanLinVector_Now(:, i)+PlatAnchorPosition_Now(:, i)), PlatAnchorPosition_Now(:, i), ...
                (Cable.TanLinVector(:, i)+Platform_Next.anchorPosition_G(:, i)), Platform_Next.anchorPosition_G(:, i)];
            
            test2 = [(TanLinVector_Now(:, j)+PlatAnchorPosition_Now(:, j)), PlatAnchorPosition_Now(:, j), ...
                (Cable.TanLinVector(:, j)+Platform_Next.anchorPosition_G(:, j)), Platform_Next.anchorPosition_G(:, j)];
            
            Ceq(index) = cableIntersectTest(test1, test2);
        end
	end
end
%力平衡等式
%构造结构矩阵（structure matrix）
A_T = zeros(6,Cable.cableNumber);
for i=1:Cable.cableNumber
    bi = Platform_Next.anchorPosition_G(:, i) - Platform_Next.pose_G(1:3);
    A_T(:, i) = [Cable.UniVector(:, i); cross(bi,Cable.UniVector(:, i))];
end

if size(tension, 2)~=1
    tension = tension';
end
Ceq(29:end) = A_T * tension + wrenchSetting.wrench;

%% 计算不等式约束中不等式左边的值
C = zeros(1, (8*obsNumber + 8+ 28 + 16)); %预分配内存加速运算：绳索到障碍物距离+绳索与动平台间的距离+绳索之间的距离+动锚点座位置限制
index = 0; %索引初始化，根据条件数叠加
% 绳索到障碍物距离
for i=1:obsNumber
    for j=1:Cable.cableNumber
        index = index + 1;
        C(index) = (obsBB(i).d_min - Cable.coDistance(i, j)) * 1e3; %<=0, , 乘1000提高精度
    end
end

% 绳索与动平台间的距离(平方)
sqr_cpDis_min = Cable.cpDis_min * Cable.cpDis_min; %平方最小允许值
for i=1:Cable.cableNumber
    index = index + 1;
    C(index) = sqr_cpDis_min - Cable.cpDistance(i); %<=0
end

% 绳索之间的距离(平方)
sqr_ccDis_min = Cable.ccDis_min * Cable.ccDis_min; %平方最小允许值
count = 0;
for i=1:Cable.cableNumber
	for j=(i+1):Cable.cableNumber
		count = count + 1;
        index = index + 1;
        C(index) = sqr_ccDis_min - Cable.ccDistance(count); %<=0
	end
end

% % 动平台到障碍物的距离
% for i=1:obsNumber
%     index = index + 1;
%     C(index) = obsBB(i).d_min - Platform_Next.poDistance(i); %<=0
% end

%动锚点座位置限制
for i=1:Cable.cableNumber
     for j=1:2
        index = index + 1;
        if j==1 %动锚点座局部位置下限
            C(index) = (MovAnchor.position_LB(i,j) - MovAnchor.positon_L(i)) * 1e3; %<=0, 乘1000提高精度
        else %动锚点座局部位置上限
            C(index) = (MovAnchor.positon_L(i) - MovAnchor.position_LB(i,j)) * 1e3; %<=0, 乘1000提高精度
        end
    end
end