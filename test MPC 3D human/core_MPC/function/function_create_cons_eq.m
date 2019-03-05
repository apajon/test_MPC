function [Aeq,beq]=function_create_cons_eq(MPC_inputs,COM_state_preview,Step_state_preview)
%% Constraints Equalities
    %TODO orientation
Aeq=[];beq=[];

%%
switch(MPC_inputs.COM_form)
    case {'poly expo'}
    otherwise
        % zeta_final=(zeta_up_ref(i+16)+zeta_down_ref(i+16))/2;
        % zeta_final=h_com/g;
        % w_final=1/sqrt(zeta_final);
        w_final=omega_temp;

        Aeq_diff_dc_ddc=[w_final*COM_state_preview.Pu_dc(end,:)+COM_state_preview.Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(COM_state_preview.Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
            zeros(1,size(COM_state_preview.Pu_dc,2)) zeros(1,size(Pu_step,2)) w_final*COM_state_preview.Pu_dc(end,:)+COM_state_preview.Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
            zeros(1,size(COM_state_preview.Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(COM_state_preview.Pu_dc,2)) zeros(1,size(Pu_step,2)) w_final*COM_state_preview.Pu_dc(end,:)+COM_state_preview.Pu_ddc(end,:)];
        beq_diff_dc_ddc=[-COM_state_preview.xf_dc(end,1);-COM_state_preview.yf_dc(end,2);-COM_state_preview.Px_dc(end,:)*MPC_inputs.c_init(:,3)]*w_final+[-COM_state_preview.xf_ddc(end,1);-COM_state_preview.yf_ddc(end,2);-COM_state_preview.Px_ddc(end,:)*MPC_inputs.c_init(:,3)];

        Aeq=Aeq_diff_dc_ddc;
        beq=beq_diff_dc_ddc;
end

%% Manage fixed foot step position
[A_fixed_step,b_fixed_step]=pankle_fixed_path(size(MPC_inputs.xstep,1),MPC_inputs.N,size(Step_state_preview.Pu_step,2),MPC_inputs.step_number_pankle_fixed);

Aeq=[Aeq;A_fixed_step];
beq=[beq;b_fixed_step];
end