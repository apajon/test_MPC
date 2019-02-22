%% Init storage QP result
% % COM along x axis
% xdddc_storage=[];
% 
% xc(1)=xcom_0(1);
% xdc(1)=xcom_0(2);
% xddc(1)=xcom_0(3);
% 
% ydddc_storage=[];
% 
% yc(1)=ycom_0(1);
% ydc(1)=ycom_0(2);
% yddc(1)=ycom_0(3);
% 
% % COM along z axis
% zdddc_storage=[];
% 
% zc(1)=zcom_0(1);
% zdc(1)=zcom_0(2);
% zddc(1)=zcom_0(3);
% 
% % foot step
% 
% if phase_type(2)=='r'
%     xstep=[xstep_l_0;xstep_r_0];
%     ystep=[ystep_l_0;ystep_r_0];
%     zstep=[zstep_l_0;zstep_r_0];
% elseif phase_type(2)=='l'
%     xstep=[xstep_r_0;xstep_l_0];
%     ystep=[ystep_r_0;ystep_l_0];
%     zstep=[zstep_r_0;zstep_l_0];
% end

%%
MPC_outputs_storage=classdef_quadratic_problem_outputs;

% COM along x axis
MPC_outputs_storage.xdddc_storage=[];

MPC_outputs_storage.xc(1)=xcom_0(1);
MPC_outputs_storage.xdc(1)=xcom_0(2);
MPC_outputs_storage.xddc(1)=xcom_0(3);

MPC_outputs_storage.ydddc_storage=[];

MPC_outputs_storage.yc(1)=ycom_0(1);
MPC_outputs_storage.ydc(1)=ycom_0(2);
MPC_outputs_storage.yddc(1)=ycom_0(3);

% COM along z axis
MPC_outputs_storage.zdddc_storage=[];

MPC_outputs_storage.zc(1)=zcom_0(1);
MPC_outputs_storage.zdc(1)=zcom_0(2);
MPC_outputs_storage.zddc(1)=zcom_0(3);

% foot step

if phase_type(2)=='r'
    MPC_outputs_storage.xstep=[xstep_l_0;xstep_r_0];
    MPC_outputs_storage.ystep=[ystep_l_0;ystep_r_0];
    MPC_outputs_storage.zstep=[zstep_l_0;zstep_r_0];
elseif phase_type(2)=='l'
    MPC_outputs_storage.xstep=[xstep_r_0;xstep_l_0];
    MPC_outputs_storage.ystep=[ystep_r_0;ystep_l_0];
    MPC_outputs_storage.zstep=[zstep_r_0;zstep_l_0];
end