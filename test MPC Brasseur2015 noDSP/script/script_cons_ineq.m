%% Constraints inequalities
    %TODO orientation
    %% Constraint ZMP in convex hull
%     no_double_support=any(phase_type_sampling_reduce~='b',2);
    no_double_support=(sum(Px_step_ref==1,2)==1);
    no_double_support=no_double_support(1+(i-1):N+(i-1),:);
    if isempty(Pu_step)
        A_zmp=[Pu_z_up(no_double_support,:);...
            -Pu_z_up(no_double_support,:);...
            Pu_z_down(no_double_support,:);...
            -Pu_z_down(no_double_support,:)];
    else
        A_zmp=[Pu_z_up(no_double_support,:) -Pu_step(no_double_support,:);...
            -Pu_z_up(no_double_support,:) Pu_step(no_double_support,:);...
            Pu_z_down(no_double_support,:) -Pu_step(no_double_support,:);...
            -Pu_z_down(no_double_support,:) Pu_step(no_double_support,:)];
    end
    
    A_zmp=blkdiag(A_zmp,A_zmp);
    
%     right_support=any(phase_type_sampling_reduce=='r',2);
%     left_support=any(phase_type_sampling_reduce=='l',2);
    if phase_type_sampling_reduce(1)=='r' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='r'
        right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
    elseif phase_type_sampling_reduce(1)=='l' || phase_type_sampling_reduce(max([1 find(phase_type_sampling_reduce~='b',1)]))=='l'
        right_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),2:2:end)==1,2),2);
        left_support=any(sum(Px_step_ref(1+(i-1):N+(i-1),1:2:end)==1,2),2);
    else
        right_support=false(size(no_double_support));
        left_support=false(size(no_double_support));
    end
        
    
    xb_zmp=[no_double_support(no_double_support)*(fronttoankle-sole_margin);...
        no_double_support(no_double_support)*(backtoankle-sole_margin);...
        no_double_support(no_double_support)*(fronttoankle-sole_margin);...
        no_double_support(no_double_support)*(backtoankle-sole_margin)]...
        +[xf_step(no_double_support,:)-xf_z_up(no_double_support,:);...
        -xf_step(no_double_support,:)+xf_z_up(no_double_support,:);...
        xf_step(no_double_support,:)-xf_z_down(no_double_support,:);...
        -xf_step(no_double_support,:)+xf_z_down(no_double_support,:)];
    
    yb_zmp=[right_support(no_double_support)*(inttoankle-sole_margin)+left_support(no_double_support)*(exttoankle-sole_margin);...
        right_support(no_double_support)*(exttoankle-sole_margin)+left_support(no_double_support)*(inttoankle-sole_margin);...
        right_support(no_double_support)*(inttoankle-sole_margin)+left_support(no_double_support)*(exttoankle-sole_margin);...
        right_support(no_double_support)*(exttoankle-sole_margin)+left_support(no_double_support)*(inttoankle-sole_margin)]...
        +[yf_step(no_double_support,:)-yf_z_up(no_double_support,:);...
        -yf_step(no_double_support,:)+yf_z_up(no_double_support,:);...
        yf_step(no_double_support,:)-yf_z_down(no_double_support,:);...
        -yf_step(no_double_support,:)+yf_z_down(no_double_support,:)];
    
    b_zmp=[xb_zmp;yb_zmp];
        
    %% Constraint foot step stretching
    A_step_stretch=[];
    b_step_stretch=[];
    xb_step_stretch=[];yb_step_stretch=[];
    
    if size(Pu_step,2)>=1
        A_step_stretch=eye(size(Pu_step,2));
        A_step_stretch(2:end,1:end-1)=A_step_stretch(2:end,1:end-1)-eye(size(Pu_step,2)-1);
        A_step_stretch=[zeros(size(Pu_step,2),N) A_step_stretch];
        A_step_stretch=[A_step_stretch;-A_step_stretch];
        
        A_step_stretch=blkdiag(A_step_stretch,A_step_stretch);
        
        %%%
        phase_type_nodouble_reduce=phase_type_reduce(any(phase_type_reduce~='b',2));
        
        if phase_type_reduce(end)=='b' && Pu_step(end,end)==0.5
            if phase_type_nodouble_reduce(end)=='r'
                phase_type_nodouble_reduce(end+1)='l';
            else
                phase_type_nodouble_reduce(end+1)='r';
            end
        end
        
        phase_type_nodouble_reduce(1)=[];
        %%%
        xb_step_stretch=[ones(size(Pu_step,2),1)*xankmax;...
            ones(size(Pu_step,2),1)*(-xankmin)];
        xb_step_stretch([1 end/2+1])=xb_step_stretch([1 end/2+1])+[xstep(end);-xstep(end)];
 
        yb_step_stretch=[any(phase_type_nodouble_reduce=='l',2)*yankmax-any(phase_type_nodouble_reduce=='r',2)*yankmin;...
            any(phase_type_nodouble_reduce=='l',2)*(-yankmin)-any(phase_type_nodouble_reduce=='r',2)*(-yankmax)];
        yb_step_stretch([1 end/2+1])=yb_step_stretch([1 end/2+1])+[ystep(end);-ystep(end)];
    end
    
    b_step_stretch=[xb_step_stretch;yb_step_stretch];
    
    % Constraint concatenation
    if i>=120
        A=[A_zmp*0;A_step_stretch];
        b=[b_zmp*0;b_step_stretch]; 
    else
        A=[A_zmp;A_step_stretch];
        b=[b_zmp;b_step_stretch]; 
    end
       
    
    A=[A zeros(size(A,1),size(H_c,2))];    
    %% Constraint vertical com motion
    %zeta_down (zddc - g) < (zc-zp) < zeta_up (zddc + g)
    A_verti_motion_up=Pu_c-diag(zeta_up_ref(1+(i-1):N+(i-1),:))*Pu_ddc;
    b_verti_motion_up=zeta_up_ref(1+(i-1):N+(i-1),:).*g+zzmp_ref_reduce-(zf_c-zeta_up_ref(1+(i-1):N+(i-1),:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);

    A_verti_motion_down=Pu_c-diag(zeta_down_ref(1+(i-1):N+(i-1),:))*Pu_ddc;
    b_verti_motion_down=zeta_down_ref(1+(i-1):N+(i-1),:).*g+zzmp_ref_reduce-(zf_c-zeta_down_ref(1+(i-1):N+(i-1),:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);

    A_verti_motion=[A_verti_motion_up;-A_verti_motion_down];
    b_verti_motion=[b_verti_motion_up;-b_verti_motion_down];

    %% COM accel z > -g
    A_verti_acc=-Pu_ddc;
    b_verti_acc=g+Px_ddc*[zc(i);zdc(i);zddc(i)];    
    
    %%
    A_verti=[A_verti_motion;A_verti_acc];
    b_verti=[b_verti_motion;b_verti_acc];
    
    A=[A;zeros(size(A_verti,1),size(A_zmp,2)) A_verti];
    b=[b;b_verti];
    
    
    %% constraint kinematics com height (polyhedron)
    Pu_diff_c_p=[Pu_c zeros(N,size(Pu_step,2))]-[zeros(N,size(Pu_c,2)) Pu_step];
    z_Pu_diff_c=Pu_c;

    xf_diff_c_p=Px_c*[xc(i);xdc(i);xddc(i)]-xf_step;
    yf_diff_c_p=Px_c*[yc(i);ydc(i);yddc(i)]-yf_step;
    zf_diff_C_p=Px_c*[zc(i);zdc(i);zddc(i)]-zzmp_ref_reduce;

%     A_diff_c_p_no_z=zeros(N*size(rot_successive,1),size(Pu_diff_c_p,2));
%     zA_diff_c_p=zeros(N*size(rot_successive,1),size(z_Pu_diff_c,2));
%     xzb_diff_c_p=zeros(N*size(rot_successive,1),1);
%     yzb_diff_c_p=zeros(N*size(rot_successive,1),1);
    A_diff_c_p_no_z=[];
    zA_diff_c_p=[];
    xzb_diff_c_p=[];
    yzb_diff_c_p=[];
    if isempty(z_Pu_diff_c)==0
        for j=1:size(rot_successive,1)
            sin_sampled=rot_successive(j,1);
            cos_sampled=rot_successive(j,2);
            polyhedron_lim_sampled=polyhedron_lim(j,1);

            
%             A_diff_c_p_no_z((1:N)+N*(j-1),:)=sin_sampled*Pu_diff_c_p;
%             zA_diff_c_p((1:N)+N*(j-1),:)=cos_sampled*z_Pu_diff_c;
            A_diff_c_p_no_z=[A_diff_c_p_no_z;sin_sampled*Pu_diff_c_p(:,:)];
            zA_diff_c_p=[zA_diff_c_p;cos_sampled*z_Pu_diff_c(:,:)];
            
%             xzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled*xf_diff_c_p-cos_sampled*zf_diff_C_p;
%             yzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled*yf_diff_c_p-cos_sampled*zf_diff_C_p;
            xzb_diff_c_p=[xzb_diff_c_p;polyhedron_lim_sampled-sin_sampled*xf_diff_c_p(:,:)-cos_sampled*zf_diff_C_p(:,:)];
            yzb_diff_c_p=[yzb_diff_c_p;polyhedron_lim_sampled-sin_sampled*yf_diff_c_p(:,:)-cos_sampled*zf_diff_C_p(:,:)];
        end
    end

    A_diff_c_p=[A_diff_c_p_no_z zeros(size(A_diff_c_p_no_z)) zA_diff_c_p;...
        zeros(size(A_diff_c_p_no_z)) A_diff_c_p_no_z zA_diff_c_p];

    b_diff_c_p=[xzb_diff_c_p;yzb_diff_c_p];

    
    if i>=120
        A=[A;A_diff_c_p];
        b=[b;b_diff_c_p];
    else
        A=[A;A_diff_c_p];
        b=[b;b_diff_c_p];
    end


%     A_height=[Pu_c;-Pu_c];
%     b_height=[0.9-zf_c;-0.7+zf_c];
% 
%     A=[A;zeros(size(A_height,1),size(A_zmp,2)) A_height];
%     b=[b;b_height];