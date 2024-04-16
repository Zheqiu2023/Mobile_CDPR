function [codis,cpdis,ccdis] = fun_update_dis(X, Platform_Next, MovAnchor, Cable, ...
                                                                     obsBB, obsNumber, vertex, face, r)
%% 初始化并记录当前点的绳长向量、绳间距离
x_step = X(1:8); %8个锚点座的步长

%% 更新动锚点座位置
MovAnchor=MovAnchor.setPosL(MovAnchor.positon_L + x_step);
%% 计算绳长向量
% 拷贝并修改自CDPRInvKine.m（只计算绳长向量TanLinVector）
RotZ = @(a) [cos(a) -sin(a) 0;
                        sin(a)  cos(a) 0;
                         0          0        1];
                     
RotX = @(a)  [1     0            0;
                         0 cos(a) -sin(a);
                         0 sin(a)  cos(a)]; 
for i=1:8
    Rotzi = MovAnchor.AnRotz_G(i);
    Ai = MovAnchor.positon_G(:,i);
    Bi = Platform_Next.anchorPosition_G(:, i); 
    %基本参数
    vec_BiAi = Ai - Bi;
    %ai = [0 0 1]'; 
    %求滑轮转角
    n1i = RotZ(Rotzi) * [1 0 0]';
    n2i = RotZ(Rotzi) * [0 1 0]';
    pulRot = atan2(vec_BiAi' * n1i, abs(vec_BiAi' * n2i));
    %求解切线绳长
    vec_AiCi_Ai = [0 r 0]'; % Ai坐标系下向量AiCi，Ci为滑轮圆心
    vec_BiAi_Ai = RotZ(Rotzi + pulRot)' * vec_BiAi; % 将向量BiAi变换到动锚点Ai坐标系
    vec_BiCi_Ai = vec_BiAi_Ai + vec_AiCi_Ai ;
    L_BiCi = norm(vec_BiCi_Ai);
    L_BiMi = sqrt(L_BiCi * L_BiCi - r * r); %切线绳长BiMi
    %求解绳索单位向量、切线向量
    theta1 = asin(r / L_BiCi);
    ui_Ai = RotX(theta1)' * vec_BiCi_Ai / L_BiCi; %Ai坐标系下绳索单位向量ui
    vec_BiMi_Ai = L_BiMi * ui_Ai; %Ai系下的绳长向量BiMi
    ui = RotZ(Rotzi + pulRot) * ui_Ai; %全局G坐标系下绳索单位向量ui
    vec_BiMi = RotZ(Rotzi + pulRot) * vec_BiMi_Ai; %全局G系下的绳长向量BiMi
    Cable.UniVector(:, i) = ui;
    Cable.TanLinVector(:, i) = vec_BiMi;
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


codis=Cable.coDistance;
cpdis=Cable.cpDistance;
ccdis=Cable.ccDistance;
                                                                 
                                                                 
                                                                 
end

