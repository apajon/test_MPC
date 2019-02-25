%% Constant
%% Mechanics
g=9.81; %m.s-1
% h_com=0.8;
% h_com=0.748964+0.095; %m
%h_com=0.78; %m


%% Create robot
robot=classdef_create_robot(robot_type);

omega_temp=sqrt(g/robot.h_com);
zeta_temp=1/omega_temp^2;

%% Create experiment
experiment=classdef_create_experiment(phase_duration_type,nb_foot_step,firstSS);