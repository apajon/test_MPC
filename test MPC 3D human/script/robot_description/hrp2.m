% HRP4 description
%% default CoM height
switch controller
    case 'vrep'
        %h_com=0.748964;
        h_com=0.780273289863387;
    case 'rviz'
        h_com=0.780678; %m
    otherwise
        msg='Bad choice of controller \n';
        msg1='vrep \n';
        msg2='rviz \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end

%% COM height limits to the floor
%with respect to h_com
h_com_max=+0.0317;
h_com_min=-0.25;

switch(walking_type)
    case 1
        h_com_max=+0.0317;
        h_com_min=-0.25;
%         h_com_max=+0.0317;
%         h_com_min=-0.25;
    case 2
        h_com_max=+0.0317;
        h_com_min=-0.25;
    case 3
        h_com_max=+0.05;
        h_com_min=-0.1;
    case 4
        h_com_max=+0.005;
        h_com_min=-0.25;
    case 5
        h_com_max=+0.05;
        h_com_min=-0.25;
end

%% Initial standing state default
switch controller
    case 'vrep'
        xcom_0=[-0.0149883888510708;0;0];
        ycom_0=[0.000850173415884411;0;0];
        zcom_0=[h_com;0;0];

        xstep_r_0=-0.0152074908954922;
        ystep_r_0=-0.0790897453289095;

        xstep_l_0=-0.0163389718197275;
        ystep_l_0=0.0816625960261456;
    case 'rviz'
        xcom_0=[-0.00365306;0;0];
        ycom_0=[+0.00047291;0;0];
        zcom_0=[h_com;0;0];

        xstep_r_0=0;
        ystep_r_0=-0.0815817;

        xstep_l_0=0;
        ystep_l_0=0.0815817;
    otherwise
        msg='Bad choice of controller \n';
        msg1='vrep \n';
        msg2='rviz \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end

%% Foot limits
backtoankle=0.1; %from back to ankle of foot
fronttoankle=0.13; %from  front to ankle of foot
exttoankle=0.075; %from exterior to ankle of foot
inttoankle=0.055; %from interior to ankle of foot

sole_margin=0.02;

%% Foot step placement limits
xankmax=0.8;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=2*inttoankle+0.0552;%0.15;%width min between ankles
% yankmin=2*inttoankle+0.0399;%0.15;%width min between ankles
% yankmin=0.1769
yankmax=2*inttoankle+0.4;%width max between ankles
% yankmax=2*inttoankle+0.0552;%width max between ankles

