%% compute linear COM and ZMP
    %% COM
    omega_temp=1;
    zeta_temp=1/omega_temp^2;
    [Px_c,Pu_c,Px_dc,Pu_dc,Px_ddc,Pu_ddc]=function_compute_com_linear_zmpVel(ones(N,1)*zeta_temp,T,N);
    
    H_c=Pu_c.'*Pu_c;
    H_dc=Pu_dc.'*Pu_dc;

    %% ZMP up
    zeta_up=zeta_up_ref(1+(i-1):N+(i-1),:);
    Px_z_up=Px_c-Px_ddc.*repmat(zeta_up,1,3);
    Pu_z_up=Pu_c-Pu_ddc.*repmat(zeta_up,1,N);
    
    %% ZMP down
    zeta_down=zeta_down_ref(1+(i-1):N+(i-1),:);
    Px_z_down=Px_c-Px_ddc.*repmat(zeta_down,1,3);
    Pu_z_down=Pu_c-Pu_ddc.*repmat(zeta_down,1,N);

%% initialization from last robot COM state
    %% COM position
    xf_c=Px_c*[xc(i);xdc(i);xddc(i)];
    yf_c=Px_c*[yc(i);ydc(i);yddc(i)];
    zf_c=Px_c*[zc(i);zdc(i);zddc(i)];
    %% COM velocity
    xf_dc=Px_dc*[xc(i);xdc(i);xddc(i)];
    yf_dc=Px_dc*[yc(i);ydc(i);yddc(i)];
    zf_dc=Px_dc*[zc(i);zdc(i);zddc(i)];
    
    %% COM acceleration
    xf_ddc=Px_ddc*[xc(i);xdc(i);xddc(i)];
    yf_ddc=Px_ddc*[yc(i);ydc(i);yddc(i)];
    zf_ddc=Px_ddc*[zc(i);zdc(i);zddc(i)];
    
    %% COM velocity ref
    xf_dc_ref=xvcom_ref(1+(i-1):N+(i-1));
    yf_dc_ref=yvcom_ref(1+(i-1):N+(i-1));
    
    %% ZMP position up
    xf_z_up=Px_z_up*[xc(i);xdc(i);xddc(i)];
    yf_z_up=Px_z_up*[yc(i);ydc(i);yddc(i)];
    
    %% ZMP position down
    xf_z_down=Px_z_down*[xc(i);xdc(i);xddc(i)];
    yf_z_down=Px_z_down*[yc(i);ydc(i);yddc(i)];


    %% ZMP mean
    xf_z_mean=(xf_z_up+xf_z_down)/2;
    yf_z_mean=(yf_z_up+yf_z_down)/2;
    
    Pu_z_mean=(Pu_z_up+Pu_z_down)/2;
    
    H_z_mean=Pu_z_mean.'*Pu_z_mean;
    
    
    %% Capture point up
    Px_Capture_up=Px_c+Px_dc.*repmat(sqrt(zeta_up_ref(1+(i-1):N+(i-1),:)),1,size(Px_dc,2));
    
    xf_Capture_up=Px_Capture_up*[xc(i);xdc(i);xddc(i)];
    yf_Capture_up=Px_Capture_up*[yc(i);ydc(i);yddc(i)];
    zf_Capture_up=Px_Capture_up*[zc(i);zdc(i);zddc(i)];
    
    Pu_Capture_up=Pu_c+Pu_dc.*repmat(sqrt(zeta_up_ref(1+(i-1):N+(i-1),:)),1,size(Pu_dc,2));
    
    %% Capture point down
    Px_Capture_down=Px_c+Px_dc.*repmat(sqrt(zeta_down_ref(1+(i-1):N+(i-1),:)),1,size(Px_dc,2));
    
    xf_Capture_down=Px_Capture_down*[xc(i);xdc(i);xddc(i)];
    yf_Capture_down=Px_Capture_down*[yc(i);ydc(i);yddc(i)];
    zf_Capture_down=Px_Capture_down*[zc(i);zdc(i);zddc(i)];
    
    Pu_Capture_down=Pu_c+Pu_dc.*repmat(sqrt(zeta_down_ref(1+(i-1):N+(i-1),:)),1,size(Pu_dc,2));
    