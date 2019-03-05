% human description
%% default CoM height
h_com=0.780678;

%% COM height limits to the floor
%with respect to h_com
h_com_max=+0.05;
h_com_min=-0.25;

%% Initial standing state default
xcom_0=[0;0;0];
ycom_0=[0;0;0];
zcom_0=[h_com;0;0];

xstep_r_0=0;
ystep_r_0=-0.0815817;

xstep_l_0=0;
ystep_l_0=0.0815817;

%% Foot limits
backtoankle=0.05; %from back to ankle of foot
fronttoankle=0.2; %from  front to ankle of foot
exttoankle=0.05; %from exterior to ankle of foot
inttoankle=0.025; %from interior to ankle of foot

sole_margin=0.002;

%% Foot step placement limits
xankmax=0.8;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=2*inttoankle+0.0552;%0.15;%width min between ankles
% yankmin=2*inttoankle+0.0399;%0.15;%width min between ankles
% yankmin=0.1769
yankmax=2*inttoankle+0.4;%width max between ankles
% yankmax=2*inttoankle+0.0552;%width max between ankles
