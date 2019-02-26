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

        Aeq_diff_dc_ddc=[w_final*Pu_dc(end,:)+Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
            zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) w_final*Pu_dc(end,:)+Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
            zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) w_final*Pu_dc(end,:)+Pu_ddc(end,:)];
        beq_diff_dc_ddc=[-xf_dc(end);-yf_dc(end);-Px_dc(end,:)*[zc(i);zdc(i);zddc(i)]]*w_final+[-xf_ddc(end);-yf_ddc(end);-Px_ddc(end,:)*[zc(i);zdc(i);zddc(i)]];

        Aeq=Aeq_diff_dc_ddc;
        beq=beq_diff_dc_ddc;
end

%% Manage fixed foot step position
[A_fixed_step,b_fixed_step]=pankle_fixed_path(size(MPC_inputs.xstep,1),MPC_inputs.N,size(Step_state_preview.Pu_step,2),MPC_inputs.step_number_pankle_fixed);

Aeq=[Aeq;A_fixed_step];
beq=[beq;b_fixed_step];