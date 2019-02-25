%% Optimization QP
MPC_outputs=classdef_quadratic_problem_outputs;

[MPC_outputs.QP_result_all,tata,MPC_outputs.converge_sampling]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);

MPC_outputs.update_properties(MPC_inputs,COM_state_preview);
