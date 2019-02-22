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
H_dz=eye(MPC_inputs.N);

%%
% add min(zeta_mean-foot_step)
% add variable part of min(com_vel-com_vel_ref)


% if i<=phase_sampling_length(3)
%     w1=10^-1;
% else
%     w1=10^-7;
% end
H_Pu=w1*H_dz+w2*H_dc;
    
%     right_support=any(phase_type_sampling_reduce=='r',2);
%     left_support=any(phase_type_sampling_reduce=='l',2);
if phase_type(2)=='r'
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
elseif phase_type(2)=='l'
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


if isempty(Pu_step)
    xH=H_Pu+...
        w3*H_z_mean;

    xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
        +w3*Pu_z_mean.'*(xf_z_mean-(xf_step+MPC_inputs.translate_step_cop_ref(1,1)));

    yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
        +w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)));
else
    xH=[H_Pu+w3*H_z_mean -w3*Pu_z_mean.'*Pu_step;...
        (-w3*Pu_z_mean.'*Pu_step).' w3*H_step];

    xf=[w2*Pu_dc.'*(xf_dc-xf_dc_ref)+...
        w3*Pu_z_mean.'*(xf_z_mean-(xf_step+MPC_inputs.translate_step_cop_ref(1,1)));...
        -w3*Pu_step.'*(xf_z_mean-(xf_step+MPC_inputs.translate_step_cop_ref(1,1)))];

    yf=[w2*Pu_dc.'*(yf_dc-yf_dc_ref)+...
        w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)));...
        -w3*Pu_step.'*(yf_z_mean-(yf_step+(left_support-right_support)*MPC_inputs.translate_step_cop_ref(1,2)))];
end

yH=xH;

H=blkdiag(xH,yH);

f=[xf;yf];
    
%%
%add Cost com heigth and min jerk
zf_c=Px_c*[zc(i);zdc(i);zddc(i)];

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

zf=w4*Pu_c.'*(zf_c-hcom_ref_max_reduce);

H=blkdiag(H,w4*H_c+w1*H_dz); % add min jerk
f=[f;zf];