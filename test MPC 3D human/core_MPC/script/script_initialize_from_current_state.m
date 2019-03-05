%Initialize the matrix for trajectories of the model from last robot state
%They are all linear function of the control variable of the
%form : c = f + Pu*X where X is the control variable and are
%decoupled along each axis

%% COM
COM_state_preview=classdef_com_state_preview(MPC_inputs);

%% CoP
CoP_state_preview=classdef_CoP_state_preview(MPC_inputs,COM_state_preview);

%% Capture point
% CP_state_preview=classdef_CapturePoint_state_preview(MPC_inputs,COM_state_preview);

%% Initialize from last fixed foot step
Step_state_preview=classdef_FootStep_state_preview(MPC_inputs,MPC_inputs.xstep,MPC_inputs.ystep);
