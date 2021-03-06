%% Constraints Equalities
    %TODO orientation
Aeq=[];beq=[];

% %%
% %COM vel
% Aeq_dc=[Pu_dc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
%     zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) Pu_dc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
%     zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) Pu_dc(end,:)*0];
% beq_dc=[-xf_dc(end);-yf_dc(end);-Px_dc(end,:)*[zc(i);zdc(i);zddc(i)]*0];
% 
% %COM acc
% Aeq_ddc=[Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_ddc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
%     zeros(1,size(Pu_ddc,2)) zeros(1,size(Pu_step,2)) Pu_ddc(end,:) zeros(1,size(Pu_step,2)) zeros(1,size(H_c,2));...
%     zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_dc,2)) zeros(1,size(Pu_step,2)) Pu_ddc(end,:)*0];
% beq_ddc=[-xf_ddc(end);-yf_ddc(end);-Px_ddc(end,:)*[zc(i);zdc(i);zddc(i)]*0];
% 
% Aeq=[Aeq_dc;Aeq_ddc];
% beq=[beq_dc;beq_ddc];
% 
% %COM heigth
% Aeq_zc=[zeros(1,size(Pu_c,2)) zeros(1,size(Pu_step,2)) zeros(1,size(Pu_c,2)) zeros(1,size(Pu_step,2)) Pu_c(end,:)];
% beq_zc=h_com+zzmp_ref_reduce(end)-Px_c(end,:)*[zc(i);zdc(i);zddc(i)];

%%
switch(COM_form)
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



        if 0 %i>=length(xvcom_ref)-39 || i==40

        else
            Aeq=Aeq_diff_dc_ddc;
            beq=beq_diff_dc_ddc;
        end
end

%% Manage fixed foot step position
[A_fixed_step,b_fixed_step]=pankle_fixed_path(size(xstep,1),N,size(Pu_step,2),step_number_pankle_fixed);

Aeq=[Aeq;A_fixed_step];
beq=[beq;b_fixed_step];