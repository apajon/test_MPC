%% update cost function
%%
% add min(zeta_mean-foot_step)
% add variable part of min(com_vel-com_vel_ref)
% xtranslate_step=(fronttoankle+backtoankle)/2-backtoankle;
xtranslate_step=0;
ytranslate_step=(exttoankle+inttoankle)/2-inttoankle;

% if i<=phase_sampling_length(2)
%     w1=10^-1;
% else
%     w1=10^-5;
% end
% H_Pu=w1*H_dddc+w2*H_dc;

%     no_double_support=any(phase_type_sampling_reduce~='b',2);
    no_double_support=(sum(Px_step_ref==1,2)==1);
    no_double_support=no_double_support(1+(i-1):N+(i-1),:);
    
%     right_support=any(phase_type_sampling_reduce=='r',2);
%     left_support=any(phase_type_sampling_reduce=='l',2);
    if phase_type(2)=='r'
        if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='r'
            left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
            right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
        elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='l'
            right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
            left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
        else
            right_support=false(size(no_double_support));
            left_support=false(size(no_double_support));
        end
    elseif phase_type(2)=='l'
        if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='r'
            right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
            left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
        elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='l'
            left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
            right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
        else
            right_support=false(size(no_double_support));
            left_support=false(size(no_double_support));
        end
    end


if isempty(Pu_step)
        xH=H_Pu+w3*H_z_mean;

        xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
            +w3*Pu_z_mean.'*(xf_z_mean-(xf_step+xtranslate_step));
        
        yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
            +w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step));
    else
        xH=[H_Pu+w3*H_z_mean -w3*Pu_z_mean.'*Pu_step;...
            (-w3*Pu_z_mean.'*Pu_step).' w3*H_step];
        
        xf=[w2*Pu_dc.'*(xf_dc-xf_dc_ref)+...
            w3*Pu_z_mean.'*(xf_z_mean-(xf_step+xtranslate_step));...
            -w3*Pu_step.'*(xf_z_mean-(xf_step+xtranslate_step))];
        
        yf=[w2*Pu_dc.'*(yf_dc-yf_dc_ref)+...
            w3*Pu_z_mean.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step));...
            -w3*Pu_step.'*(yf_z_mean-(yf_step+(left_support-right_support)*ytranslate_step))];
    end
    
    yH=xH;
    
    H=blkdiag(xH,yH);
    
    f=[xf;yf];
    
%%
%add Cost com heigth and min jerk
    zf_c=Px_c*[zc(i);zdc(i);zddc(i)];

    zzmp_ref_reduce=zzmp_ref(1+(i-1):N+(i-1),:);
    hcom_ref_reduce=hcom_ref(1+(i-1):N+(i-1),:);
    hcom_ref_max_reduce=hcom_ref_max(1+(i-1):N+(i-1),:);
%     if i<10
%         hcom_ref_reduce=0.8;
%     elseif i<120
%         hcom_ref_reduce=0.95;%h_com+h_com_max;%hcom_ref(1+(i-1):N+(i-1),:);
%     else
%         hcom_ref_reduce=0.8;
%     end

    zf=w4*Pu_c.'*(zf_c-zzmp_ref_reduce-hcom_ref_max_reduce);

    H=blkdiag(H,w4*H_c+10^-5*w1*H_dddc); % add min jerk
    f=[f;zf];