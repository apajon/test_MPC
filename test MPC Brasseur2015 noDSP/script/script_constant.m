%% Constant
%% Mechanics
g=9.81; %m.s-1
% h_com=0.8;
h_com=0.780678; %m

%% Sampling time
% T=5*10^-3;
% N=300;

% T=5*10^-2;
% N=30;

T=0.1;
N=16;

%% Initial Robot State
% xcom_0=[0;0;0];
xcom_0=[-0.00365306;0;0];

% ycom_0=[0;0;0];
ycom_0=[+0.00047291;0;0];

zcom_0=[h_com;0;0];

xstep_r_0=0;
% ystep_r_0=-0.13;
ystep_r_0=-0.0815817;

xstep_l_0=0;
% ystep_l_0=+0.13;
ystep_l_0=0.0815817;

%% Foot limits
% %hrp2
% backtoankle=0.1; %from back to ankle of foot
% fronttoankle=0.13; %from  front to ankle of foot
% exttoankle=0.075; %from exterior to ankle of foot
% inttoankle=0.055; %from interior to ankle of foot

%hrp4
backtoankle=0.098; %from back to ankle of foot
fronttoankle=0.128; %from  front to ankle of foot
exttoankle=0.076; %from exterior to ankle of foot
inttoankle=0.054; %from interior to ankle of foot   

sole_margin=0.02;

%% Foot step placement limits
xankmax=0.4;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=2*inttoankle+0.05;%0.15;%width min between ankles
yankmax=2*inttoankle+0.4;%width max between ankles

%% Phase duration
phase_duration_r=0.7;
phase_duration_l=0.7;
phase_duration_b=0.1;
phase_duration_start=2.4;
phase_duration_stop=2.4;

%% COM height limits to the floor
switch(walking_type)
    case 1
        h_com_max=+0.05;
        h_com_min=-0.1;
    case 2
        h_com_max=+0.05;
        h_com_min=-0.25;
    case 3
        h_com_max=+0.05;
        h_com_min=-0.1;
end
