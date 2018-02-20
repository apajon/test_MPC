%% Constant
%% Mechanics
g=9.81; %m.s-1
h_com=0.8; %m

%% Sampling time
T=5*10^-3;
N=300;

T=5*10^-2;
N=30;

T=0.1;
N=16;

%% Initial Robot State
xcom_0=[0;0;0];
ycom_0=[0;0;0];
zcom_0=[h_com;0;0];

xstep_r_0=0;
ystep_r_0=-0.13;

xstep_l_0=0;
ystep_l_0=+0.13;

%% Foot limits
backtoankle=0.1; %from back to ankle of foot
fronttoankle=0.13; %from  front to ankle of foot
exttoankle=0.075; %from exterior to ankle of foot
inttoankle=0.055; %from interior to ankle of foot

%% Foot step placement limits
xankmax=0.4;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=2*inttoankle+0.13;%width min between ankles
yankmax=2*inttoankle+0.4;%width max between ankles

%% Phase duration
phase_duration_r=0.7;
phase_duration_l=0.7;
phase_duration_b=0.1;
phase_duration_start=2.4;
phase_duration_stop=2.4;

%% COM height limits to the floor
h_com_max=+0.05;
h_com_min=-0.01;