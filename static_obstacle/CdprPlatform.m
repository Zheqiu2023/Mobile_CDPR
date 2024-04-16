classdef CdprPlatform 
    %末端动平台类
    %   Detailed explanation goes here
    
    properties (Access = public)
        vertexPosition_P; 
        vertexPosition_G;
        anchorPosition_P;
        anchorPosition_G;
        anchorDis;
        anchorUniVec_P;
        pose_G;
        originPose_G;
        boundary;
        poDistance;
    end
    
    methods (Access = public)
        function obj = CdprPlatform()
            vertexPosition_P = [25  25 -28;
                                25 -25 -28;
                               -25  25 -28;
                               -25 -25 -28;
                                25  25  30;
                                25 -25  30;
                               -25  25  30;
                               -25 -25  30]'*1e-3; % 3X8, 动平台各顶点位置（P-coordinate）
            obj.vertexPosition_P = vertexPosition_P;
            save('vertexPosition_P.mat', 'vertexPosition_P');

            anchorPosition_P = vertexPosition_P(5:8, :); % 3X4, P-coordinate anchor positions
            obj.anchorPosition_P = anchorPosition_P;
            save('anchorPosition_P.mat', 'anchorPosition_P');

            obj.vertexPosition_G = zeros(3,8); % 3X8, predefine G-coordinate vertex positions
            obj.anchorPosition_G = zeros(3,4); % 3X4, predefine G-coordinate anchor positions 
            load("anchorDistance.mat");
            obj.anchorDis = anchorDistance; % 1X6, load pre-calculated distance between each anchor points
            load("anchorUniVec_P.mat");
            obj.anchorUniVec_P = anchorUniVec_P; % 3X6, load pre-calculated unit vector between each anchor points 
            obj.originPose_G = [0 0 28 0 0 0]'*1e-3; % 6X1, original global center position and Z-X-Y eular angle
            obj.pose_G = obj.originPose_G; % 6X1, real time global center position and Z-X-Y eular angle
            obj.boundary = [-500 500;
                            -500 500;
                             28  1000] * 1e-3; % 3X2, platform position boundary, m
            obj.poDistance = [ ];
        end
        
        function obj=setPoseG(obj, pose_G)
            if size(pose_G, 1)~=6
                pose_G = pose_G';
            end
            obj.pose_G = pose_G;
            %update Platform anchors' and vertices' G-coordinate
            T_GP = trans(obj.pose_G);
            temPosition_P = cat(1, obj.anchorPosition_P, ones(1,4));
            temPosition_G = T_GP * temPosition_P;
            obj.anchorPosition_G = temPosition_G(1:3,:);
            temPosition_P = cat(1, obj.vertexPosition_P, ones(1,4));
            temPosition_G = T_GP * temPosition_P;
            obj.vertexPosition_G = temPosition_G(1:3,:);
        end
        
        function obj=resetPoseG(obj)
            %reset platform pose to the original one
            obj = obj.setPoseG(obj.originPose_G);
        end

    end
end

