clear all;
close all;
clc;

pkg load symbolic;

%% Flat displaing
sympref display flat

addpath ./dh_basics;

%%% VARIABLES (CPU)
syms TX;    % Robot localization position
syms TY;    % Robot localization position
syms TZ;    % Robot localization position
syms QX;    % Robot localization orientation
syms QY;    % Robot localization orientation
syms QZ;    % Robot localization orientation
syms QW;    % Robot localization orientation

syms TH2;   %LiDAR tower angle from center

% TX = 0;    % Robot localization position
% TY = 0;    % Robot localization position
% TZ = 0;    % Robot localization position
% QX = 0;    % Robot localization orientation
% QY = 0;    % Robot localization orientation
% QZ = 0;    % Robot localization orientation
% QW = 1;    % Robot localization orientation
%
% TH2 = 0;   %LiDAR tower angle from center

%%% VARIABLES (GPU)
syms TH5;   %LiDAR scan angle from center
syms A5;    %LiDAR scan range

% TH5 = 0;
% A5 = 4;


%%% CONSTANTS
syms a1;    % Offset from rover center to LiDAR tower Z-axis
syms d2;    % Height from rover center to LiDAR scanner
syms al3;   % Angle of LiDAR tilt in its Y-axis

% a1 = -0.3;
% d2 = 0.7;
% al3 = 0.2;

%%% DH MATRIXES
A_ODOM = [];
A_ROVER = [];
A_SCAN = [];





%%% STAGE 1
printf("\n\n================================================\n")
printf("STAGE 1 - Calculating World to Rover matrix (A_ODOM)\n\n")

A_ODOM = [  [1 - 2*QY*QY - 2*QZ*QZ,   2*QX*QY - 2*QZ*QW,       2*QX*QZ + 2*QY*QW,       TX]
            [2*QX*QY + 2*QZ*QW,       1 - 2*QX*QX - 2*QZ*QZ,   2*QY*QZ - 2*QX*QW,       TY]
            [2*QX*QZ - 2*QY*QW,       2*QY*QZ + 2*QX*QW,       1 - 2*QX*QX - 2*QY*QY,   TZ]
            [       0,                       0,                        0,                1]  ];





%%% STAGE 2
printf("\n\n================================================\n")
printf("STAGE 2 - Calculating Rover to LiDAR matrix (A_ROVER)\n\n")

A_R1 = transX(a1);
A_R2 = rotZ(TH2) * transZ(d2);
A_R3 = rotZ(pi/2) * rotX(al3);
A_R4 = rotZ(-pi/2);

A_ROVER = A_R1 * A_R2 * A_R3 * A_R4;




%%% STAGE 3
printf("\n\n================================================\n")
printf("STAGE 3 - Calculating LiDAR to Scan matrix (A_SCAN)\n\n")


A_S1 = rotZ(TH5);
A_S2 = transX(A5);

A_SCAN = A_S1 * A_S2;




%%% STAGE 4
printf("\n\n================================================\n")
printf("STAGE 4 - Calculating World to Scan matrix (A)\n\n")

A = A_ODOM * A_ROVER * A_SCAN




%%% STAGE 5
printf("\n\n================================================\n")
printf("STAGE 5 - Extracting new point's position (A)\n\n")

new_x = A(1,4);
new_y = A(2,4);
new_z = A(3,4);



%%% STAGE 6
printf("\n\n================================================\n")
printf("STAGE 6 CPU calculations\n\n")

A_CPU = A_ODOM * A_ROVER;



%%% STAGE 7
printf("\n\n================================================\n")
printf("STAGE 7 GPU calculations\n\n")

syms A_CPU_0;
syms A_CPU_1;
syms A_CPU_2;
syms A_CPU_3;
syms A_CPU_4;
syms A_CPU_5;
syms A_CPU_6;
syms A_CPU_7;
syms A_CPU_8;
syms A_CPU_9;
syms A_CPU_10;
syms A_CPU_11;
syms A_CPU_12;
syms A_CPU_13;
syms A_CPU_14;
syms A_CPU_15;

A_CPU_SYM = [ [A_CPU_0,  A_CPU_1,  A_CPU_2,  A_CPU_3 ]
              [A_CPU_4,  A_CPU_5,  A_CPU_6,  A_CPU_7 ]
              [A_CPU_8,  A_CPU_9,  A_CPU_10, A_CPU_11]
              [A_CPU_12, A_CPU_13, A_CPU_14, A_CPU_15] ];

A_GPU = A_CPU_SYM * A_SCAN;




%%% STAGE 8
printf("\n\n================================================\n")
printf("STAGE 8 output\n\n")

printf("\n\n===================== CPU =====================\n")


for i=1:4
    for j=1:4
        printf('\n\nA_CPU_%d%d\n', i, j)
        printf('dk_cpu.m[%d][%d] = ', i, j)
        disp(A_CPU(i,j))
    end
end

printf("\n\n===================== GPU =====================\n")

% for i=1:3
%     for j=4:4
%         printf('\n\nA_GPU_%d%d\n', i, j)
%         disp(A_GPU(i,j))
%     end
% end

        printf('\n\nA_GPU_%d%d\n', 1, 4)
        printf('point.x = ')
        disp(A_GPU(1,4))

        printf('\n\nA_GPU_%d%d\n', 2, 4)
        printf('point.y = ')
        disp(A_GPU(2,4))

        printf('\n\nA_GPU_%d%d\n', 3, 4)
        printf('point.z = ')
        disp(A_GPU(3,4))
