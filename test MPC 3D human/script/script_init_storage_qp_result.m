%% Initialize MPC storage of MPC iteration from QP result
MPC_outputs_storage=classdef_quadratic_problem_outputs;

% COM along x axis
MPC_outputs_storage.xdddc_storage=[];

MPC_outputs_storage.xc(1)=robot.xcom_0(1);
MPC_outputs_storage.xdc(1)=robot.xcom_0(2);
MPC_outputs_storage.xddc(1)=robot.xcom_0(3);

MPC_outputs_storage.ydddc_storage=[];

MPC_outputs_storage.yc(1)=robot.ycom_0(1);
MPC_outputs_storage.ydc(1)=robot.ycom_0(2);
MPC_outputs_storage.yddc(1)=robot.ycom_0(3);

% COM along z axis
MPC_outputs_storage.zdddc_storage=[];

MPC_outputs_storage.zc(1)=robot.zcom_0(1);
MPC_outputs_storage.zdc(1)=robot.zcom_0(2);
MPC_outputs_storage.zddc(1)=robot.zcom_0(3);

% foot step

if experiment.phase_type(2)=='r'
    MPC_outputs_storage.xstep=[robot.xstep_l_0;robot.xstep_r_0];
    MPC_outputs_storage.ystep=[robot.ystep_l_0;robot.ystep_r_0];
    MPC_outputs_storage.zstep=[experiment.zstep_l_0;experiment.zstep_r_0];
elseif experiment.phase_type(2)=='l'
    MPC_outputs_storage.xstep=[robot.xstep_r_0;robot.xstep_l_0];
    MPC_outputs_storage.ystep=[robot.ystep_r_0;robot.ystep_l_0];
    MPC_outputs_storage.zstep=[experiment.zstep_r_0;experiment.zstep_l_0];
end