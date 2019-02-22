clear all

clc

addpath script/ function/ classdef/

robot_type='human';
%hrp4
%human

phase_duration_type=1;

%% Sampling time
% T=5*10^-3;
% N=300;

% T=5*10^-2;
% N=30;

N_r=7;
N_l=7;
N_b=1;
N_start=14*3;
N_stop=24;
% N=N_r+N_l+N_b*2;

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

save_figure=false;

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM vel ref
run('script/script_ref.m')

%% ZMP COM Linear form
COM_form='poly expo'
%'com jerk' : COM with piece-wise jerk
%'zmp vel' : ZMP with piece-wise velocity
%'poly expo' : 2nd poly of exponential

%% Polyhedron limits
kinematic_limit='hexagonTranslation';
%'' : very simple polyhedron
%'hexagon' : hexagon kinematic limits
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
for i=1:phase_duration_iteration_cumul(end)
    i
    if i>=48
        i
    end
    
    N=1;
    t=phase_duration_sampling(i);
    while(t<preview_windows_duration)
        t=t+phase_duration_sampling(i+N);
        N=N+1;
    end
    
    preview_windows=1+(i-1):N+(i-1);
%     preview_windows=1+(i):N+(i);
    %%
    run('script/script_problem_iteration_creation.m')
    
    %% Initialization from last robot state
    run('script/script_initialize_from_current_state.m')
    
    %% Cost
%     switch(COM_form)
%         case {'com jerk','poly expo'}
%             run('script/script_update_cost_comJerk.m')
%         case 'zmp vel'
%             run('script/script_update_cost_zmpVel.m')
% %         case 'poly expo'
% %             error('script_update_cost_polyExpo.m not define')
%         otherwise
%             error('Bad COM_form')
%     end
    
    run('script/script_update_cost.m')
    
    
    %% Constraint Inequalities
%     switch(COM_form)
%         case 'com jerk'
%             run('script/script_cons_ineq_comJerk.m')
%         case 'zmp vel'
%             run('script/script_cons_ineq_zmpVel.m')
%     end
    run('script/script_cons_ineq_zmpVel.m')
    
    %% Constraint Equalities
%     switch(COM_form)
%         case 'com jerk'
%             run('script/script_cons_eq_comJerk.m')
%         case 'zmp vel'
%             run('script/script_cons_eq_zmpVel.m')
%     end
    run('script/script_cons_eq_comJerk.m')
    
    %% constraints
    lb=[];ub=[];x0=[];

    %% Options
%     options=optimoptions('quadprog','Display','iter','MaxIterations',1000);
    options=optimoptions('quadprog','Display','final');
%     options=optimoptions('quadprog','Display','off');

    %% Optimization QP
    run('script/script_optimization_Qp.m')
%     
%     figure(1)
%     clf
%     title('MPC COM trajectory')
%     xlabel('t [s]') % x-axis label
%     ylabel('position [m]') % y-axis label
%     axis([0 16 -0.5 4])
%     hold on
%     plot([0;phase_duration_sampling_cumul(1:i)],xc(1:i+1),'b')
%     plot([0;phase_duration_sampling_cumul(1:i)],yc(1:i+1),'g')
%     plot([0;phase_duration_sampling_cumul(1:i)],zc(1:i+1),'k')
%     plot(phase_duration_sampling_cumul(i:i+N-1),xf_c+Pu_c*QP_result_all{i}(1:N),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),yf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N),'r')
%     legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
%     hold off
%     
%     figure(2)
%     clf
%     title('MPC CoP trajectory')
%     xlabel('t [s]') % x-axis label
%     ylabel('position [m]') % y-axis label
%     axis([0 16 -0.5 4])
%     hold on
%     plot([0;phase_duration_sampling_cumul(1:i)],1*xc+0*xdc-(zc-zzmp_ref(1:i+1))./(zddc+g).*xddc,'b')
%     plot([0;phase_duration_sampling_cumul(1:i)],1*yc+0*ydc-(zc-zzmp_ref(1:i+1))./(zddc+g).*yddc,'g')
%     plot([0;phase_duration_sampling_cumul(1:i)],zzmp_ref(1:i+1),'k')
%     plot(phase_duration_sampling_cumul(i:i+N-1),xf_c+Pu_c*QP_result_all{i}(1:N)-...
%         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
%         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
%         (xf_ddc+Pu_ddc*QP_result_all{i}(1:N)),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),yf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)-...
%         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
%         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
%         (yf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),zzmp_ref(i+1:i+N),'r')
%     legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
%     hold off

    
    figure(1)
    clf
    title('MPC COM trajectory')
    xlabel('t [s]') % x-axis label
    ylabel('position [m]') % y-axis label
    axis([0 16 -0.5 4])
    hold on
    plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.xc(1:i+1),'b')
    plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.yc(1:i+1),'g')
    plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.zc(1:i+1),'k')
    plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,1)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(1:N),'r')
    plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,2)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r')
    plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,3)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N),'r')
    legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
    hold off
    
    if movie_record
        F_COM=getframe(figure(1));
        F_CoP=getframe(figure(2));
        
        writeVideo(v_COM,F_COM);
        writeVideo(v_CoP,F_CoP);
    end
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