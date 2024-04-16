function [X, Y, Z] = fit_archor_coor(x)
%% 用以下数据拟合出合适的曲线，得到下面的公式
% step = [0:0.004:0.504]';  % 每个4mm标定一个锚点座坐标
% lf_archor_x = readmatrix('archor-calibration_coor.xlsx','Sheet','lf','Range','A:A');
% lf_archor_y = readmatrix('archor-calibration_coor.xlsx','Sheet','lf','Range','B:B');
% lf_archor_z = readmatrix('archor-calibration_coor.xlsx','Sheet','lf','Range','C:C');
% rf_archor_x = readmatrix('archor-calibration_coor.xlsx','Sheet','rf','Range','A:A');
% rf_archor_y = readmatrix('archor-calibration_coor.xlsx','Sheet','rf','Range','B:B');
% rf_archor_z = readmatrix('archor-calibration_coor.xlsx','Sheet','rf','Range','C:C');
% lb_archor_x = readmatrix('archor-calibration_coor.xlsx','Sheet','lb','Range','A:A');
% lb_archor_y = readmatrix('archor-calibration_coor.xlsx','Sheet','lb','Range','B:B');
% lb_archor_z = readmatrix('archor-calibration_coor.xlsx','Sheet','lb','Range','C:C');
% rb_archor_x = readmatrix('archor-calibration_coor.xlsx','Sheet','rb','Range','A:A');
% rb_archor_y = readmatrix('archor-calibration_coor.xlsx','Sheet','rb','Range','B:B');
% rb_archor_z = readmatrix('archor-calibration_coor.xlsx','Sheet','rb','Range','C:C');

% lf
p1 = -38; p2 = 34.41; p3 = -0.7488; p4 = -7.503; p5 = 2.112; p6 = 0.517; p7 = 1128;
X(1) = p1*x(1)^6 + p2*x(1)^5 + p3*x(1)^4 + p4*x(1)^3 + p5*x(1)^2 + p6*x(1) + p7;
p1 = -17.24; p2 = 50.67; p3 = -42.05; p4 = 14.76; p5 = -3.018; p6 = 11.02; p7 = 445.8;
Y(1) = p1*x(1)^6 + p2*x(1)^5 + p3*x(1)^4 + p4*x(1)^3 + p5*x(1)^2 + p6*x(1) + p7;
p1 = -224.2; p2 = 349; p3 = -204.7; p4 = 56.1; p5 = -7.405; p6 = 1001; p7 = -770.2;
Z(1) = p1*x(1)^6 + p2*x(1)^5 + p3*x(1)^4 + p4*x(1)^3 + p5*x(1)^2 + p6*x(1) + p7;
    
% rf
p1 = 272.7; p2 = -380.9; p3 = 190.3; p4 = -38.37; p5 = 1.971; p6 = -8.085; p7 = 1136;
X(2) = p1*x(2)^6 + p2*x(2)^5 + p3*x(2)^4 + p4*x(2)^3 + p5*x(2)^2 + p6*x(2) + p7;
p1 = 151.7; p2 = -214.8; p3 = 108.2; p4 = -21.45; p5 = 1.437; p6 = -4.608; p7 = -639.1;
Y(2) = p1*x(2)^6 + p2*x(2)^5 + p3*x(2)^4 + p4*x(2)^3 + p5*x(2)^2 + p6*x(2) + p7;
p1 = 279.5; p2 = -426.2; p3 = 244.6; p4 = -66.83; p5 = 8.934; p6 = 1000; p7 = -768.6;
Z(2) = p1*x(2)^6 + p2*x(2)^5 + p3*x(2)^4 + p4*x(2)^3 + p5*x(2)^2 + p6*x(2) + p7;
    
% lb
p1 = 101.7; p2 = -152.2; p3 = 83.16; p4 = -19.9; p5 = 1.876; p6 = -0.347; p7 = 11.3;
X(3) = p1*x(3)^6 + p2*x(3)^5 + p3*x(3)^4 + p4*x(3)^3 + p5*x(3)^2 + p6*x(3) + p7;
p1 = -27.03; p2 = 52.27; p3 = -36.47; p4 = 10.5; p5 = -1.221; p6 = -8.238; p7 = 438.1;
Y(3) = p1*x(3)^6 + p2*x(3)^5 + p3*x(3)^4 + p4*x(3)^3 + p5*x(3)^2 + p6*x(3) + p7;
p1 = 184.7; p2 = -293.6; p3 = 185.1; p4 = -60.11; p5 = 10.68; p6 = 999.8; p7 = -769.6;
Z(3) = p1*x(3)^6 + p2*x(3)^5 + p3*x(3)^4 + p4*x(3)^3 + p5*x(3)^2 + p6*x(3) + p7;
    
% rb
p1 = 1.808; p2 = -0.5859; p3 = -4.588; p4 = 4.351; p5 = -1.144; p6 = 2.893; p7 = 15.06;
X(4) = p1*x(4)^6 + p2*x(4)^5 + p3*x(4)^4 + p4*x(4)^3 + p5*x(4)^2 + p6*x(4) + p7;
p1 = 186.4; p2 = -271.6; p3 = 147.4; p4 = -37.05; p5 =  4.125; p6 = 4.691; p7 = -654.9;
Y(4) = p1*x(4)^6 + p2*x(4)^5 + p3*x(4)^4 + p4*x(4)^3 + p5*x(4)^2 + p6*x(4) + p7; 
p1 = 135.2; p2 = -178.1; p3 = 84.91; p4 = -17.21; p5 = 0.9634; p6 = 1001; p7 = -767.9;
Z(4) = p1*x(4)^6 + p2*x(4)^5 + p3*x(4)^4 + p4*x(4)^3 + p5*x(4)^2 + p6*x(4) + p7;

X = X*1e-3;
Y = Y*1e-3;
Z = Z*1e-3;