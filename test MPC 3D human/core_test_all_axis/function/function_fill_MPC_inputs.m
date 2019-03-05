function [MPC_inputs]=function_fill_MPC_inputs(robot,experiment,MPC_outputs_storage,...
    COM_form,kinematic_limit,cop_ref_type,firstSS,...
    k)

MPC_inputs=classdef_MPC_problem_inputs();
%%
k
if k>=48
    k
end

N=1;
t=experiment.phase_duration_sampling(k);
while(t<experiment.preview_windows_duration)
    t=t+experiment.phase_duration_sampling(k+N);
    N=N+1;
end

preview_windows=1+(k-1):N+(k-1);
%     preview_windows=1+(k):N+(k);

%%
MPC_inputs.g=experiment.g;

MPC_inputs.omega_temp=experiment.omega_temp;

MPC_inputs.N=N;
MPC_inputs.phase_duration_sampling=experiment.phase_duration_sampling(preview_windows,:);

MPC_inputs.zeta_temp=ones(N,1)*experiment.zeta_temp;
MPC_inputs.zeta_up=experiment.zeta_up_ref(preview_windows,:);
MPC_inputs.zeta_down=experiment.zeta_down_ref(preview_windows,:);

MPC_inputs.c_init=[MPC_outputs_storage.xc(k) MPC_outputs_storage.yc(k) MPC_outputs_storage.zc(k);...
                MPC_outputs_storage.xdc(k) MPC_outputs_storage.ydc(k) MPC_outputs_storage.zdc(k);...
                MPC_outputs_storage.xddc(k) MPC_outputs_storage.yddc(k) MPC_outputs_storage.zddc(k)];
            
MPC_inputs.dc_ref=[experiment.vcom_ref(preview_windows,1) experiment.vcom_ref(preview_windows,2)];

MPC_inputs.Px_step=experiment.Px_step_ref(preview_windows,:);
MPC_inputs.Px_step=MPC_inputs.Px_step(:,any(MPC_inputs.Px_step));

MPC_inputs.phase_type_sampling=experiment.phase_type_sampling(preview_windows,:);

MPC_inputs.no_double_support=(sum(experiment.Px_step_ref==1,2)==1);
MPC_inputs.no_double_support=MPC_inputs.no_double_support(preview_windows,:);

MPC_inputs.backtoankle=robot.backtoankle;
MPC_inputs.fronttoankle=robot.fronttoankle;
MPC_inputs.exttoankle=robot.exttoankle;
MPC_inputs.inttoankle=robot.inttoankle;

MPC_inputs.sole_margin=robot.sole_margin;

MPC_inputs.COM_form=COM_form;

MPC_inputs.kinematic_limit=kinematic_limit;
MPC_inputs.plan_hexagon=experiment.plan_hexagon;
MPC_inputs.z_leg_min=experiment.z_leg_min;
MPC_inputs.z_decalage_tot=experiment.z_decalage_tot;

MPC_inputs.phase_type_reduce=MPC_inputs.phase_type_sampling(([0;MPC_inputs.phase_type_sampling(1:end-1)]==MPC_inputs.phase_type_sampling)==0);

MPC_inputs.OptimCostWeight=experiment.OptimCostWeight;

MPC_inputs.xankmax=robot.xankmax;
MPC_inputs.xankmin=robot.xankmin;
MPC_inputs.yankmax=robot.yankmax;
MPC_inputs.yankmin=robot.yankmin;

if MPC_inputs.phase_type_reduce(1)=='b' || MPC_inputs.phase_type_reduce(1)=="start" || MPC_inputs.phase_type_reduce(1)=="stop"
    MPC_inputs.xstep=MPC_outputs_storage.xstep(end-1:end,1);
    MPC_inputs.ystep=MPC_outputs_storage.ystep(end-1:end,1);
    MPC_inputs.zstep=MPC_outputs_storage.zstep(end-1:end,1);
else
    MPC_inputs.xstep=MPC_outputs_storage.xstep(end,1);
    MPC_inputs.ystep=MPC_outputs_storage.ystep(end,1);
    MPC_inputs.zstep=MPC_outputs_storage.zstep(end,1);
end
% MPC_inputs.xstep=MPC_outputs_storage.xstep;
% MPC_inputs.ystep=MPC_outputs_storage.ystep;
% MPC_inputs.zstep=MPC_outputs_storage.zstep;

MPC_inputs.nbKnownSteps=size(MPC_outputs_storage.xstep,1);

if isempty(experiment.step_number_pankle_fixed)
    MPC_inputs.step_number_pankle_fixed=[0 MPC_outputs_storage.xstep(2) MPC_outputs_storage.ystep(2)];
else
    MPC_inputs.step_number_pankle_fixed=experiment.step_number_pankle_fixed;
end

MPC_inputs.zfloor_ref_reduce=experiment.zfloor_ref(preview_windows,:);
% MPC_inputs.hcom_ref_reduce=hcom_ref(preview_windows,:);
MPC_inputs.hcom_ref_max_reduce=experiment.hcom_ref_max(preview_windows,:);

MPC_inputs.no_end_constraint=size(experiment.vcom_ref,1)-39;

MPC_inputs.yaw=experiment.yaw_sampling(preview_windows,:);

%% experiment.zfloor_ref from ankle position to CoP reference along axis [x y]
switch(cop_ref_type)
    case 'ankle_center'
        MPC_inputs.translate_step_cop_ref=[0 0];
    case 'foot_center'
        MPC_inputs.translate_step_cop_ref=[(MPC_inputs.fronttoankle+MPC_inputs.backtoankle)/2-MPC_inputs.backtoankle ...
            (MPC_inputs.exttoankle+MPC_inputs.inttoankle)/2-MPC_inputs.inttoankle];
%     case 'waist_center'
    otherwise
        msg='Bad choice of cop_ref_type \n';
        msg1='ankle_center \n';
        msg2='foot_center \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end

%% translation from ankle position to polyhedron of COM reachable region center along axis [x y]
MPC_inputs.translate_step_polyhedron_type=experiment.translate_step_polyhedron_type;

%%
if firstSS=='r'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='r'
        left_support=any(sum(experiment.Px_step_ref(preview_windows,1:2:end)==1,2),2);
        right_support=any(sum(experiment.Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='l'
        right_support=any(sum(experiment.Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support=any(sum(experiment.Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
elseif firstSS=='l'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='r'
        right_support=any(sum(experiment.Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support=any(sum(experiment.Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='l'
        left_support=any(sum(experiment.Px_step_ref(preview_windows,2:2:end)==1,2),2);
        right_support=any(sum(experiment.Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
end

if k>1 && (MPC_inputs.phase_type_sampling(1)=='b'|MPC_inputs.phase_type_sampling(1)=="start"|MPC_inputs.phase_type_sampling(1)=="stop")
    if experiment.phase_type_sampling(k-1)=='r'
        right_support(1)=true;
    elseif experiment.phase_type_sampling(k-1)=='l'
        left_support(1)=true;
    end
end

MPC_inputs.right_support=right_support;
MPC_inputs.left_support=left_support;

if any(experiment.Px_step_ref(1+(k-1),1:2:end)==0.5,2)||any(experiment.Px_step_ref(N+(k-1),1:2:end)==0.5,2)
    double_support=any(experiment.Px_step_ref(preview_windows,1:2:end)==0.5,2);
else
    double_support=false;
end

MPC_inputs.double_support=double_support;

no_double_support_capture=(sum(experiment.Px_step_ref==1,2)==1);
no_double_support_capture=no_double_support_capture(preview_windows,:);
no_double_support_capture(1:end-1,:)=[no_double_support_capture(1:end-1,:)==2];

MPC_inputs.no_double_support_capture=no_double_support_capture;

end