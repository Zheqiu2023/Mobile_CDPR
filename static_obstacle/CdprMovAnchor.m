classdef CdprMovAnchor
    %动锚点座类
    %   Detailed explanation goes here
    
    properties (Access = public)
        positon_origin_G = [0.557  0.545 0.546;
                            0.557 -0.545 0.546;
                           -0.557  0.545 0.546;
                           -0.557 -0.545 0.546]';  % 3X4,global origin position,unit:m                                       
        positon_G ; % 3X4,global realtime position,unit:m
        positon_L = zeros(1,4); % 1X4,local realtime position,unit:m
        position_LB = zeros(4,2); % 4X2,local position boundary ,unit:m

        AnRotz_G = pi/180.*[0 180 180 0]; % 1X4, anchor Z-rotation angle relative to G-coordinate，unit:rad
        PulRotz_L = zeros(1,4); % 1X4, pully Z-rotation angle relative to local coordinate，unit:rad
        PulCenter = zeros(3,4); % 3X4, predefine pully center G-coordinate

        caliPosition_L = [0.546 0.546 0.546 0.546];   % 1X4, local linear rail calibration position，unit:m
        endPosition_L = [1.04 1.04 1.04 1.04];   % 1X4, local linear rail end position，unit:m
        caliPosition_G;
        endPosition_G;
    end
    
    methods (Access = public)
        function  obj=CdprMovAnchor()
            %constructor
            obj.positon_G = obj.positon_origin_G;
            obj.caliPosition_G = obj.positon_origin_G;
            obj.endPosition_G = cat(1, obj.positon_origin_G(1:2, :), endPosition_L);
            
            obj.position_LB(:, 2) = obj.endPosition_L' - obj.caliPosition_L'; %upper boundary of local position 
            obj.position_LB(:, 1) = [0 0 0 0]'; %lower boundary of local position
        end
        
        function obj=setPosL(obj, posL)
            %set local position 
            if size(posL, 1)~=1
                posL = posL';
            end
            obj.positon_L = posL;
            %update global position of movable anchors by local position
            obj.positon_G(3, :) = obj.positon_origin_G(3, :) + obj.positon_L;
        end
        
        function obj=resetAllPos(obj)
            %reset all anchor position to origin
            obj.positon_G = obj.positon_origin_G;
            obj.positon_L = zeros(1,4);
        end
    end
end

