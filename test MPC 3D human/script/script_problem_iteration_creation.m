%%
i
if i>=48
    i
end

N=1;
t=phase_duration_sampling(i);
while(t<experiment.preview_windows_duration)
    t=t+phase_duration_sampling(i+N);
    N=N+1;
end

preview_windows=1+(i-1):N+(i-1);
%     preview_windows=1+(i):N+(i);

%%
MPC_inputs.g=g;

MPC_inputs.omega_temp=omega_temp;

MPC_inputs.N=N;
MPC_inputs.phase_duration_sampling=phase_duration_sampling(preview_windows,:);

MPC_inputs.zeta_temp=ones(N,1)*zeta_temp;
MPC_inputs.zeta_up=zeta_up_ref(preview_windows,:);
MPC_inputs.zeta_down=zeta_down_ref(preview_windows,:);

MPC_inputs.c_init=[MPC_outputs_storage.xc(i) MPC_outputs_storage.yc(i) MPC_outputs_storage.zc(i);...
                MPC_outputs_storage.xdc(i) MPC_outputs_storage.ydc(i) MPC_outputs_storage.zdc(i);...
                MPC_outputs_storage.xddc(i) MPC_outputs_storage.yddc(i) MPC_outputs_storage.zddc(i)];
            
MPC_inputs.dc_ref=[xvcom_ref(preview_windows,:) yvcom_ref(preview_windows,:)];

MPC_inputs.Px_step=Px_step_ref(preview_windows,:);
MPC_inputs.Px_step=MPC_inputs.Px_step(:,any(MPC_inputs.Px_step));

MPC_inputs.phase_type_sampling=experiment.phase_type_sampling(preview_windows,:);

MPC_inputs.no_double_support=(sum(Px_step_ref==1,2)==1);
MPC_inputs.no_double_support=MPC_inputs.no_double_support(preview_windows,:);

MPC_inputs.backtoankle=robot.backtoankle;
MPC_inputs.fronttoankle=robot.fronttoankle;
MPC_inputs.exttoankle=robot.exttoankle;
MPC_inputs.inttoankle=robot.inttoankle;

MPC_inputs.sole_margin=robot.sole_margin;

MPC_inputs.COM_form=COM_form;

MPC_inputs.kinematic_limit=kinematic_limit;
MPC_inputs.plan_hexagon=plan_hexagon;
MPC_inputs.z_leg_min=z_leg_min;
MPC_inputs.z_decalage_tot=z_decalage_tot;

MPC_inputs.phase_type_reduce=MPC_inputs.phase_type_sampling(([0;MPC_inputs.phase_type_sampling(1:end-1)]==MPC_inputs.phase_type_sampling)==0);

MPC_inputs.OptimCostWeight=OptimCostWeight;

MPC_inputs.xankmax=robot.xankmax;
MPC_inputs.xankmin=robot.xankmin;
MPC_inputs.yankmax=robot.yankmax;
MPC_inputs.yankmin=robot.yankmin;

MPC_inputs.xstep=MPC_outputs_storage.xstep;
MPC_inputs.ystep=MPC_outputs_storage.ystep;
MPC_inputs.zstep=MPC_outputs_storage.zstep;

if isempty(step_number_pankle_fixed)
    MPC_inputs.step_number_pankle_fixed=[0 MPC_outputs_storage.xstep(2) MPC_outputs_storage.ystep(2)];
else
    MPC_inputs.step_number_pankle_fixed=step_number_pankle_fixed;
end

MPC_inputs.zzmp_ref_reduce=zzmp_ref(preview_windows,:);
% MPC_inputs.hcom_ref_reduce=hcom_ref(preview_windows,:);
MPC_inputs.hcom_ref_max_reduce=hcom_ref_max(preview_windows,:);

MPC_inputs.no_end_constraint=length(xvcom_ref)-39;

%% translation from ankle position to CoP reference along axis [x y]
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
switch(polyhedron_type)
    case 'ankle_center'
         translation_x=0;
         translation_y=0;
         MPC_inputs.translate_step_polyhedron_type=[0 0];
    case 'foot_center'
         translation_x=(fronttoankle+backtoankle)/2-backtoankle;
         translation_y=-((exttoankle+inttoankle)/2-inttoankle);
         MPC_inputs.translate_step_polyhedron_type=[(fronttoankle+backtoankle)/2-backtoankle ...
             -((exttoankle+inttoankle)/2-inttoankle)];
    case 'waist_center'
         translation_x=0;
         translation_y=0.06845;
         MPC_inputs.translate_step_polyhedron_type=[0 0.06845];
     otherwise
        msg='Bad choice of polyhedron_type \n';
        msg1='ankle_center \n';
        msg2='foot_center \n';
        msg3='waist_center \n';
        errormsg=[msg msg1 msg2 msg3];
        error(errormsg,[])
end

%%
if firstSS=='r'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='r'
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='l'
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
elseif firstSS=='l'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='r'
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling=='r'|MPC_inputs.phase_type_sampling=='l',1)]))=='l'
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
end

if i>1 && (MPC_inputs.phase_type_sampling(1)=='b'|MPC_inputs.phase_type_sampling(1)=="start"|MPC_inputs.phase_type_sampling(1)=="stop")
    if experiment.phase_type_sampling(i-1)=='r'
        right_support(1)=true;
    elseif experiment.phase_type_sampling(i-1)=='l'
        left_support(1)=true;
    end
end

MPC_inputs.right_support=right_support;
MPC_inputs.left_support=left_support;

if any(Px_step_ref(1+(i-1),1:2:end)==0.5,2)||any(Px_step_ref(N+(i-1),1:2:end)==0.5,2)
    double_support=any(Px_step_ref(preview_windows,1:2:end)==0.5,2);
else
    double_support=false;
end

MPC_inputs.double_support=double_support;

no_double_support_capture=(sum(Px_step_ref==1,2)==1);
no_double_support_capture=no_double_support_capture(preview_windows,:);
no_double_support_capture(1:end-1,:)=[no_double_support_capture(1:end-1,:)==2];

MPC_inputs.no_double_support_capture=no_double_support_capture;

clear left_support right_support double_support no_double_support_capture

%%
inputs_properties=properties(MPC_inputs);
for k=1:size(inputs_properties,1)
    if isempty(MPC_inputs.([inputs_properties{k}]))
        msg='Error in inputs construction \n';
        msg1=['inputs.' inputs_properties{k} ' is empty \n'];
        errormsg=[msg msg1];
        error(errormsg,[])
    end
end
clear inputs_properties
