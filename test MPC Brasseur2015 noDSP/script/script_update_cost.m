%% update cost function
%%
% add min(zeta_mean-foot_step)
% add variable part of min(com_vel-com_vel_ref)

if isempty(Pu_step)
        xH=H_Pu+w3*H_z_mean;

        xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
            +w3*Pu_z_mean.'*(xf_z_mean-xf_step);
        
        yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
            +w3*Pu_z_mean.'*(yf_z_mean-yf_step);
    else
        xH=[H_Pu+w3*H_z_mean -w3*Pu_z_mean.'*Pu_step;...
            (-w3*Pu_z_mean.'*Pu_step).' w3*H_step];
        
        xf=[w2*Pu_dc.'*(xf_dc-xf_dc_ref)+...
            w3*Pu_z_mean.'*(xf_z_mean-xf_step);...
            -w3*Pu_step.'*(xf_z_mean-xf_step)];
        
        yf=[w2*Pu_dc.'*(yf_dc-yf_dc_ref)+...
            w3*Pu_z_mean.'*(yf_z_mean-yf_step);...
            -w3*Pu_step.'*(yf_z_mean-yf_step)];
    end
    
    yH=xH;
    
    H=blkdiag(xH,yH);
    
    f=[xf;yf];
    
%%
%add Cost com heigth and min jerk
    zf_c=Px_c*[zc(i);zdc(i);zddc(i)];

    zzmp_ref_reduce=zzmp_ref(1+(i-1):N+(i-1),:);
    if i<10
        hcom_ref_reduce=0.8;
    elseif i<120
        hcom_ref_reduce=0.95;%h_com+h_com_max;%hcom_ref(1+(i-1):N+(i-1),:);
    else
        hcom_ref_reduce=0.8;
    end

    zf=w4*Pu_c.'*(zf_c-zzmp_ref_reduce-hcom_ref_reduce);

    H=blkdiag(H,w4*H_c+w1*H_dddc); % add min jerk
    f=[f;zf];