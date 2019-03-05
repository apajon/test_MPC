%Compute the Cost function to minimize by the optimization solver
%The problem is: min(QP_result_all)
%COM trajectory control variable
%+distance between COM velocity and a velocity reference
%+distance between CoP_mean and a position reference
%+distance between COM vertical position and a height reference
%
%As any trajectory can be written in a linear way with QP_result_all, the
%problem is written as a quadratic problem and solve with a Qp
%
%As the problem is decoupled along each axis:
%QP_result_all is build as [X;Y;Z] where X, Y and Z are the vector of
%control variable along respectively, x, y and z-axis.
%X is build as [Xc;Xstep] where Xc is a vector of COM control variable
%along the preview window and Xstep is a vector of foot step position
%along the preview window
%Y is build as X
%Z is only a vector of COM control variable along the preview window

%%
%min COM traj control variable
Cost_minCOM_control=classdef_CostPart(eye(MPC_inputs.N),[],zeros(MPC_inputs.N,1),0);

% min com vel to ref
xCost_minCOM_VelToRef=classdef_CostPart(COM_state_preview.Pu_dc,[],COM_state_preview.f_dc(:,1),MPC_inputs.dc_ref(:,1));
yCost_minCOM_VelToRef=classdef_CostPart(COM_state_preview.Pu_dc,[],COM_state_preview.f_dc(:,2),MPC_inputs.dc_ref(:,2));

% min mean CoP to ref
xCost_minZMPmean_PosToRef=classdef_CostPart(CoP_state_preview.Pu_z_mean,...
    Step_state_preview.Pu_step,...
    CoP_state_preview.f_z_mean(:,1),...
    Step_state_preview.f_step(:,1)+MPC_inputs.translate_step_cop_ref(1,1)...
    );

yCost_minZMPmean_PosToRef=classdef_CostPart(CoP_state_preview.Pu_z_mean,...
    Step_state_preview.Pu_step,...
    CoP_state_preview.f_z_mean(:,2),...
    Step_state_preview.f_step(:,2)+(MPC_inputs.left_support-MPC_inputs.right_support)*MPC_inputs.translate_step_cop_ref(1,2)...
    );

% min COM height to ref
zCost_minCOM_HeightToRef=classdef_CostPart(COM_state_preview.Pu_c,...
    [],...
    COM_state_preview.f_c(:,3),...
    MPC_inputs.hcom_ref_max_reduce...
    );

%% Concatenate Cost function independently along each axis
xH=MPC_inputs.OptimCostWeight(1)*blkdiag(Cost_minCOM_control.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +MPC_inputs.OptimCostWeight(2)*blkdiag(xCost_minCOM_VelToRef.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +MPC_inputs.OptimCostWeight(3)*xCost_minZMPmean_PosToRef.H;

xf=MPC_inputs.OptimCostWeight(1)*[Cost_minCOM_control.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +MPC_inputs.OptimCostWeight(2)*[xCost_minCOM_VelToRef.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +MPC_inputs.OptimCostWeight(3)*xCost_minZMPmean_PosToRef.f;

yH=MPC_inputs.OptimCostWeight(1)*blkdiag(Cost_minCOM_control.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +MPC_inputs.OptimCostWeight(2)*blkdiag(yCost_minCOM_VelToRef.H,zeros(size(Step_state_preview.Pu_step,2)))...
    +MPC_inputs.OptimCostWeight(3)*yCost_minZMPmean_PosToRef.H;

yf=MPC_inputs.OptimCostWeight(1)*[Cost_minCOM_control.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +MPC_inputs.OptimCostWeight(2)*[yCost_minCOM_VelToRef.f;zeros(size(Step_state_preview.Pu_step,2),1)]...
    +MPC_inputs.OptimCostWeight(3)*yCost_minZMPmean_PosToRef.f;

zH=MPC_inputs.OptimCostWeight(1)*Cost_minCOM_control.H...
    +MPC_inputs.OptimCostWeight(4)*zCost_minCOM_HeightToRef.H;

zf=MPC_inputs.OptimCostWeight(1)*Cost_minCOM_control.f...
    +MPC_inputs.OptimCostWeight(4)*zCost_minCOM_HeightToRef.f;

%% Concatenate Cost function along all axis
H=blkdiag(xH,yH);
H=blkdiag(H,zH);

f=[xf;yf;zf];
