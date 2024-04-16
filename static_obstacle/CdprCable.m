classdef CdprCable
    %绳缆类
    %   Detailed explanation goes here
    
    properties (Access = public)
        cableNumber = 4;
        E = 150*1e9; % young's modulus, Pa
        A = 0.25*0.25*pi*1e-6; % area of cable ,m^2
        Fixed_Length = [507.22, 507.22, 507.22, 507.22]*1e-3; % length of fixed cable segment(winch -> A_i), m
        Length = zeros(1,cableNumber); % predefine real-time cable length(free segment, A_i -> B_i), m
        OriginLength = zeros(1,cableNumber); %original cable length(free segment), set as the reference to calculate winch motors position, m
        UniVector = zeros(3,cableNumber); % predefine real time cable unit vector(dirction)
        UniVector_P = zeros(3,cableNumber); % predefine real time cable unit vector(dirction)，P-coordinate
        TanLinVector = zeros(3,cableNumber); % predefine real time line vector tangent to pully
        TanLinVector_P = zeros(3,cableNumber); % predefine real time line vector tangent to pully，P-coordinate
        K = zeros(1,cableNumber); % predefine real time cable stiffness, N/m
        voronoi = cell(2, cableNumber); % cell array defining the voronoi region of each cable
        ccDistance = zeros(1,nchoosek(cableNumber,2)); % 1X10, calculated real-time distance between each cables
        cpDistance = zeros(1, cableNumber); % 1X4，distance between cable and platform
        coDistance; %1Xn，distance between cable and obstacles
        ccDis_min; %allowed mininum distance between cables
        ccDis_Intersect; %do intersection test in the next step if distance between two cable less than ccDis_Intersect in current step
        cpDis_min; %allowed mininum distance between cable and platform
        tensionDistrib = zeros(cableNumber, 1); %4X1，real-time tension force distribution in 4 cables
    end
    
    methods (Access = public)
        function obj = CdprCable(ccDis_min, ccDis_Intersect, cpDis_min)
           %init cable class
           if nargin==3
            obj.ccDis_min = ccDis_min;
            obj.ccDis_Intersect = ccDis_Intersect;
            obj.cpDis_min = cpDis_min;
           end
        end
        
    end
end

