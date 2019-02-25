clear all

clc

addpath script/ function/ classdef/

%%
robot_type='human';
%hrp4
%human

phase_duration_type='phase_duration_01';

walking_type=1;
% 1 : walking flat
% 2 : walking airbus stairs
% 3 : walking flat quick
% 4 : walking flat fixed foot step positions
% 5 : walking airbus stairs fixed foot step positions

controller='vrep';
% vrep
% rviz

cop_ref_type='ankle_center';
%'ankle_center' : polyhedron centered on the ankle
%'foot_center' : polyhedron centered on the middle of the foot  

polyhedron_type='waist_center';
%'ankle_center' : polyhedron centered on the ankle
%'foot_center' : polyhedron centered on the middle of the foot
%'waist_center' : polyhedron centered on the middle of the waist   

kinematic_limit='hexagonTranslation';
%'' : very simple polyhedron
%'hexagon' : hexagon kinematic limits
%'hexagonTranslation' : hexagon kinematic limits with translation

COM_form='poly expo' 
%'com jerk' : COM with piece-wise jerk
%'zmp vel' : ZMP with piece-wise velocity
%'poly expo' : 2nd poly of exponential

%b = both feet; r = right foot; l = left foot
nb_foot_step=4;
firstSS='r';

save_figure=false;

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM vel ref
run('script/script_ref.m')


%% Polyhedron limits
switch kinematic_limit
    case ''
        number_level=[];
        run('script/script_polyhedron.m')
    case 'hexagon'
        number_level=2;
        run('script/script_polyhedron_hexagon.m')
    case 'hexagonTranslation'
        number_level=2;
        run('script/script_polyhedron_hexagon_translation.m')
    otherwise
        error('choose a type of kinematic_limit')
end

%% Init storage QP result
run('script/script_init_storage_qp_result.m')


%% Precomputation
%as Camille
w1=10^-7; %jerk -7
w2=10^0; %com vel ref
switch(walking_type)
    case {1,2,3}
        w2=10^0; %com vel ref
    case {4,5}
        w2=10^0; %com vel ref
end
w3=10^-1; %zmp wth zeta mean close to step
w4=10^-2; %com height

w5=10^-1*0; %zmp acceleration aka COM acceleration
w6=10^-2*0; %zmp vel between segment

OptimCostWeight=[w1 w2 w3 w4 w5 w6];

%% movie record
converge_sampling=[];
QP_result_all=[];
movie_record=false;
if movie_record  
    v_COM = VideoWriter('COM_MPC.avi');
    v_COM.Quality = 95;
    v_COM.FrameRate=10;
    open(v_COM);
    
    v_CoP = VideoWriter('CoP_MPC.avi');
    v_CoP.Quality = 95;
    v_CoP.FrameRate=10;
    open(v_CoP);
end
%% Optimization problem QP
% Sampling update
tic
for i=1:experiment.phase_duration_iteration_cumul(end)
    %% creation of inputs of a MPC iteration 
    MPC_inputs=classdef_quadratic_problem_inputs;
    run('script/script_problem_iteration_creation.m')
    
    %% linear MPC iteration
    MPC_outputs=function_MPC_iteration(MPC_inputs);
    
    %%
    if false
        run('script/script_display_online.m')
    end 
    
    %% Store MPC results
    MPC_outputs_storage.add_storage(MPC_outputs);

    run('script/script_movie_record.m')
end
toc

if movie_record
    close(v_COM)
    close(v_CoP)
end

%% Results ZMP
run('script/script_zmp.m')

%% foot traj in the air
% hstep_move=0.05;
hstep_move=0.2;
run('script/script_foot_traj_air.m')
% run('script/script_foot_traj_air_stairs.m')

%% discretization trajectories
run('script/script_traj_discretization.m')

%% 
dt_type_phase_=any(phase_type_sampling_enlarge=='l',2)*1+any(phase_type_sampling_enlarge=='r',2)*2;
dt_type_phase_=[0;dt_type_phase_];

%% Plot results
run('script/script_plot_results.m')
    
%% write txt
if false  
    run('script/script_write_txt.m')
end
%%
if save_figure
    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D'])
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta'])
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal'])
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal'])
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView'])

    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D']...
        ,'png')
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta']...
        ,'png')
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal']...
        ,'png')
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal' ]...
        ,'png')
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView' ]...
        ,'png')
end