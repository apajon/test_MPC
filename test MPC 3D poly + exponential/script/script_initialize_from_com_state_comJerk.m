%% initialization from last robot COM state
    %% COM position
    xf_c=Px_c*[xc(i);xdc(i);xddc(i)];
    yf_c=Px_c*[yc(i);ydc(i);yddc(i)];
    zf_c=Px_c*[zc(i);zdc(i);zddc(i)];
    %% COM velocity
    xf_dc=Px_dc*[xc(i);xdc(i);xddc(i)];
    yf_dc=Px_dc*[yc(i);ydc(i);yddc(i)];
    
    %% COM velocity ref
    xf_dc_ref=xvcom_ref(1+(i-1):N+(i-1));
    yf_dc_ref=yvcom_ref(1+(i-1):N+(i-1));
    
    %% COM acceleration
    xf_ddc=Px_ddc*[xc(i);xdc(i);xddc(i)];
    yf_ddc=Px_ddc*[yc(i);ydc(i);yddc(i)];
    
    
    %% ZMP
%     xf_z=Px_z*[xc(i);xdc(i);xddc(i)];
%     yf_z=Px_z*[yc(i);ydc(i);yddc(i)];
    
    Px_z=Px_z_noheight;
    Px_z(:,3)=Px_z(:,3)-h_com/g;
    
    xf_z=Px_z*[xc(i);xdc(i);xddc(i)];
    yf_z=Px_z*[yc(i);ydc(i);yddc(i)];
    
    Pu_z=Pu_z_noheight-tril(ones(size(Pu_z_noheight)))*T*h_com/g;
    
%     H_z=Pu_z.'*Pu_z;
    
    %% ZMP up
    Px_z_up=Px_z_noheight;
    Px_z_up(:,3)=Px_z_up(:,3)-zeta_up_ref(1+(i-1):N+(i-1),:);
    
    xf_z_up=Px_z_up*[xc(i);xdc(i);xddc(i)];
    yf_z_up=Px_z_up*[yc(i);ydc(i);yddc(i)];
    
    Pu_z_up=Pu_z_noheight-tril(ones(size(Pu_z_noheight)))*T*diag(zeta_up_ref(1+(i-1):N+(i-1),:));
    
%     H_z=Pu_z.'*Pu_z;
    
    %% ZMP down
    Px_z_down=Px_z_noheight;
    Px_z_down(:,3)=Px_z_down(:,3)-zeta_down_ref(1+(i-1):N+(i-1),:);
    
    xf_z_down=Px_z_down*[xc(i);xdc(i);xddc(i)];
    yf_z_down=Px_z_down*[yc(i);ydc(i);yddc(i)];
    
    Pu_z_down=Pu_z_noheight-tril(ones(size(Pu_z_noheight)))*T*diag(zeta_down_ref(1+(i-1):N+(i-1),:));
    
%     H_z=Pu_z.'*Pu_z;

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
    