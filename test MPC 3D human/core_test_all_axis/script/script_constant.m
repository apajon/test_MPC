%% Constant

%% Create robot
robot=classdef_create_robot(robot_type);

%% Create experiment
experiment=classdef_create_experiment();

optimWeight_filename='config/optim_weight/optim_weight_01.m';
experiment.classdef_create_experiment_test(phase_duration_type,...
    nb_foot_step,firstSS,...
    kinematic_limit,robot,polyhedron_position,...
    optimWeight_filename,...
    walking_type);