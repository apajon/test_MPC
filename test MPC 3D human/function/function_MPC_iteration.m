function [MPC_outputs]=function_MPC_iteration(MPC_inputs)

%% Initialization from last robot state
run('script_initialize_from_current_state.m')
    
%% Cost
run('script_update_cost.m')


%% Constraint Inequalities
run('script_cons_ineq_zmpVel.m')

%% Constraint Equalities
run('script_cons_eq_comJerk.m')

%% constraints
lb=[];ub=[];x0=[];

%% Optimization QP
%     options=optimoptions('quadprog','Display','iter','MaxIterations',1000);
options=optimoptions('quadprog','Display','final');
%     options=optimoptions('quadprog','Display','off');

run('script_optimization_Qp.m')

end