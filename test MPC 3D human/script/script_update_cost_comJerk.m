%% update cost function
% min com pos to ref
H_c=COM_state_preview.Pu_c.'*COM_state_preview.Pu_c;
% min com vel to ref
H_dc=COM_state_preview.Pu_dc.'*COM_state_preview.Pu_dc;
% min mean CoP to ref
H_z_mean=CoP_state_preview.Pu_z_mean.'*CoP_state_preview.Pu_z_mean;
% step ref
H_step=Step_state_preview.Pu_step.'*Step_state_preview.Pu_step;

% min com Jerk
H_dddc=eye(MPC_inputs.N);

% min com acc
H_ddc=COM_state_preview.Pu_ddc.'*COM_state_preview.Pu_ddc;

% min zmp vel
Pu_dz=CoP_state_preview.Pu_dz_mean(2:end,:)-CoP_state_preview.Pu_dz_mean(1:end-1,:);
H_dz=Pu_dz.'*Pu_dz;

xf_dz=CoP_state_preview.f_dz_mean(2:end,1)-CoP_state_preview.f_dz_mean(1:end-1,1);
yf_dz=CoP_state_preview.f_dz_mean(2:end,2)-CoP_state_preview.f_dz_mean(1:end-1,2);

%%
% add min(zeta_mean-foot_step)
% add variable part of min(com_vel-com_vel_ref)

% if i<=phase_sampling_length(3)
%     w1=10^-1;
% else
%     w1=10^-7;
% end
H_Pu=w1*H_dddc+w2*H_dc+w5*H_ddc+w6*H_dz;



%     right_support=any(MPC_inputs.phase_type_sampling=='r',2);
%     left_support=any(MPC_inputs.phase_type_sampling=='l',2);

firstSS=min(find(phase_type=='r',1),find(phase_type=='l',1));
if phase_type(firstSS)=='r'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='r'
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='l'
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
elseif phase_type(firstSS)=='l'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='r'
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='l'
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
end


if isempty(Step_state_preview.Pu_step)
    xH=H_Pu+w3*H_z_mean;

    xf=w2*COM_state_preview.Pu_dc.'*(COM_state_preview.f_dc(:,1)-MPC_inputs.dc_ref(:,1))...
        +w3*CoP_state_preview.Pu_z_mean.'*(CoP_state_preview.f_z_mean(:,1)-(Step_state_preview.f_step(:,1)+MPC_inputs.translate_step_cop_ref(1,1)))...
        +w5*COM_state_preview.Pu_ddc.'*COM_state_preview.f_ddc(:,1)...
        +w6*Pu_dz.'*xf_dz;

    yf=w2*COM_state_preview.Pu_dc.'*(COM_state_preview.f_dc(:,2)-MPC_inputs.dc_ref(:,2))...
        +w3*CoP_state_preview.Pu_z_mean.'*(CoP_state_preview.f_z_mean(:,2)-(Step_state_preview.f_step(:,2)+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)))...
        +w5*COM_state_preview.Pu_ddc.'*COM_state_preview.f_ddc(:,2)...
        +w6*Pu_dz.'*yf_dz;
else
    xH=[H_Pu+w3*H_z_mean -w3*CoP_state_preview.Pu_z_mean.'*Step_state_preview.Pu_step;...
        (-w3*CoP_state_preview.Pu_z_mean.'*Step_state_preview.Pu_step).' w3*H_step];

    xf=[w2*COM_state_preview.Pu_dc.'*(COM_state_preview.f_dc(:,1)-MPC_inputs.dc_ref(:,1))+...
        w3*CoP_state_preview.Pu_z_mean.'*(CoP_state_preview.f_z_mean(:,1)-(Step_state_preview.f_step(:,1)+MPC_inputs.translate_step_cop_ref(1,1)))+...
        w5*COM_state_preview.Pu_ddc.'*COM_state_preview.f_ddc(:,1)+...
        w6*Pu_dz.'*xf_dz; ...
        -w3*Step_state_preview.Pu_step.'*(CoP_state_preview.f_z_mean(:,1)-(Step_state_preview.f_step(:,1)+MPC_inputs.translate_step_cop_ref(1,1)))];

    yf=[w2*COM_state_preview.Pu_dc.'*(COM_state_preview.f_dc(:,2)-MPC_inputs.dc_ref(:,2))+...
        w3*CoP_state_preview.Pu_z_mean.'*(CoP_state_preview.f_z_mean(:,2)-(Step_state_preview.f_step(:,2)+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)))+...
        +w5*COM_state_preview.Pu_ddc.'*COM_state_preview.f_ddc(:,2)+...
        w6*Pu_dz.'*yf_dz;...
        -w3*Step_state_preview.Pu_step.'*(CoP_state_preview.f_z_mean(:,2)-(Step_state_preview.f_step(:,2)+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)))];
end

yH=xH;

H=blkdiag(xH,yH);

f=[xf;yf];
    
%%
%add Cost com heigth and min jerk
zf_c=COM_state_preview.Px_c*[zc(i);zdc(i);zddc(i)];

zzmp_ref_reduce=zzmp_ref(preview_windows,:);
hcom_ref_reduce=hcom_ref(preview_windows,:);
hcom_ref_max_reduce=hcom_ref_max(preview_windows,:);
%     if i<10
%         hcom_ref_reduce=0.8;
%     elseif i<120
%         hcom_ref_reduce=0.95;%h_com+h_com_max;%hcom_ref(preview_windows,:);
%     else
%         hcom_ref_reduce=0.8;
%     end

% zf=w4*COM_state_preview.Pu_c.'*(zf_c-zzmp_ref_reduce-hcom_ref_max_reduce);
zf=w4*COM_state_preview.Pu_c.'*(zf_c-hcom_ref_max_reduce);

zH=w1*H_dddc+w4*H_c;

H=blkdiag(H,zH); % add min jerk
% H=blkdiag(H,w4*H_c+10^-7*H_dddc); % add min jerk
f=[f;zf];

%%
%min COM traj control input
Cost_minCOM_control=classdef_CostPart(eye(MPC_inputs.N),[],zeros(N,1),0);

% min com vel to ref
xCost_minCOM_VelToRef=classdef_CostPart(COM_state_preview.Pu_dc,[],COM_state_preview.f_dc(:,1),MPC_inputs.dc_ref(:,1));
yCost_minCOM_VelToRef=classdef_CostPart(COM_state_preview.Pu_dc,[],COM_state_preview.f_dc(:,2),MPC_inputs.dc_ref(:,2));

% min mean CoP to ref
firstSS=min(find(phase_type=='r',1),find(phase_type=='l',1));
if phase_type(firstSS)=='r'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='r'
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='l'
        right_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
elseif phase_type(firstSS)=='l'
    if MPC_inputs.phase_type_sampling(1)=='r' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='r'
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
    elseif MPC_inputs.phase_type_sampling(1)=='l' || MPC_inputs.phase_type_sampling(max([1 find(MPC_inputs.phase_type_sampling~='b',1)]))=='l'
        left_support=any(sum(Px_step_ref(preview_windows,2:2:end)==1,2),2);
        right_support=any(sum(Px_step_ref(preview_windows,1:2:end)==1,2),2);
    else
        right_support=false(size(MPC_inputs.no_double_support));
        left_support=false(size(MPC_inputs.no_double_support));
    end
end

xCost_minZMPmean_PosToRef=classdef_CostPart(CoP_state_preview.Pu_z_mean,...
    Step_state_preview.Pu_step,...
    CoP_state_preview.f_z_mean(:,1),...
    Step_state_preview.f_step(:,1)+MPC_inputs.translate_step_cop_ref(1,1)...
    );

yCost_minZMPmean_PosToRef=classdef_CostPart(CoP_state_preview.Pu_z_mean,...
    Step_state_preview.Pu_step,...
    CoP_state_preview.f_z_mean(:,2),...
    Step_state_preview.f_step(:,2)+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)...
    );

% min COM height to ref
hcom_ref_max_reduce=hcom_ref_max(preview_windows,:);

zCost_minCOM_HeightToRef=classdef_CostPart(COM_state_preview.Pu_c,...
    [],...
    COM_state_preview.f_c(:,3),...
    hcom_ref_max_reduce...
    );

%%
xH_=OptimCostWeight(1)*blkdiag(Cost_minCOM_control.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +OptimCostWeight(2)*blkdiag(xCost_minCOM_VelToRef.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +OptimCostWeight(3)*xCost_minZMPmean_PosToRef.H;

xf_=OptimCostWeight(1)*[Cost_minCOM_control.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +OptimCostWeight(2)*[xCost_minCOM_VelToRef.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +OptimCostWeight(3)*xCost_minZMPmean_PosToRef.f;

yH_=OptimCostWeight(1)*blkdiag(Cost_minCOM_control.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +OptimCostWeight(2)*blkdiag(yCost_minCOM_VelToRef.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +OptimCostWeight(3)*yCost_minZMPmean_PosToRef.H;

yf_=OptimCostWeight(1)*[Cost_minCOM_control.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +OptimCostWeight(2)*[yCost_minCOM_VelToRef.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +OptimCostWeight(3)*yCost_minZMPmean_PosToRef.f;

zH_=OptimCostWeight(1)*Cost_minCOM_control.H...
    +OptimCostWeight(4)*zCost_minCOM_HeightToRef.H;

zf_=OptimCostWeight(1)*Cost_minCOM_control.f...
    +OptimCostWeight(4)*zCost_minCOM_HeightToRef.f;
%%
if xH~=xH_
    msg = 'Error occurred.';
    error(msg)
end

if yH~=yH_
    msg = 'Error occurred.';
    error(msg)
end

if zH~=zH_
    msg = 'Error occurred.';
    error(msg)
end

if xf~=xf_
    msg = 'Error occurred.';
    error(msg)
end

if yf~=yf_
    msg = 'Error occurred.';
    error(msg)
end

if zf~=zf_
    msg = 'Error occurred.';
    error(msg)
end