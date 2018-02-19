clear all
clc

addpath script/ function/

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM ref
run('script/script_ref.m')

%% ZMP COM Linear form
run('script/script_zmp_com_linear_form.m')

%% Polyhedron limits
run('script/script_polyhedron.m')

%% Init storage QP result
run('script/script_init_storage_qp_result.m')


%% Precomputation
% same along x and y axis
w1=10^-6;
w2=10^-2;
w3=1;
w4=10^-6;

% min Jerk
H_dddc=eye(N);

% min com pos to ref
H_c=Pu_c.'*Pu_c;

% min com vel to ref
H_dc=Pu_dc.'*Pu_dc;

% Preconcatenate H
H_Pu=w1*H_dddc+w2*H_dc;


%% Optimization problem QP
% Sampling update
tic
for i=1:33%round(max(phase_duration_cumul)/T)
    i
    %% COM velocity
    xf_dc=Px_dc*[xc(i);xdc(i);xddc(i)];
    yf_dc=Px_dc*[yc(i);ydc(i);yddc(i)];
    
    %% COM velocity ref
    xf_dc_ref=xvcom_ref(1+(i-1):N+(i-1));
    yf_dc_ref=yvcom_ref(1+(i-1):N+(i-1));
    
    
%     %% ZMP
%     xf_z=Px_z*[xc(i);xdc(i);xddc(i)];
%     yf_z=Px_z*[yc(i);ydc(i);yddc(i)];
    
    %% ZMP up and down
    % ZMP up
    Px_z_up=Px_z;
    Px_z_up(:,3)=Px_z_up(:,3)-zeta_up_ref(1+(i-1):N+(i-1),:);
    
    xf_z_up=Px_z_up*[xc(i);xdc(i);xddc(i)];
    yf_z_up=Px_z_up*[yc(i);ydc(i);yddc(i)];
    
    Pu_z_up=Pu_z;
    Pu_z_up=Pu_z_up-tril(ones(size(Pu_z_up)))*diag(zeta_up_ref(1+(i-1):N+(i-1),:));
    
    % ZMP down
    Px_z_down=Px_z;
    Px_z_down(:,3)=Px_z_down(:,3)-zeta_down_ref(1+(i-1):N+(i-1),:);
    
    xf_z_down=Px_z_down*[xc(i);xdc(i);xddc(i)];
    yf_z_down=Px_z_down*[yc(i);ydc(i);yddc(i)];
    
    Pu_z_down=Pu_z;
    Pu_z_down=Pu_z_down-tril(ones(size(Pu_z_up)))*diag(zeta_down_ref(1+(i-1):N+(i-1),:));
    
    % ZMP mean
    Pu_z_mean=(Pu_z_up+Pu_z_down)/2;
    
    xf_z_mean=(xf_z_up+xf_z_down)/2;
    yf_z_mean=(yf_z_up+yf_z_down)/2;
    
    H_Pu_z_mean=Pu_z_mean.'*Pu_z_mean;
    
    %% Foot step ref
    Px_step_reduce=Px_step_ref(1+(i-1):N+(i-1),:);
    Px_step_reduce=Px_step_reduce(:,any(Px_step_reduce));
    
    phase_type_sampling_reduce=phase_type_sampling(i:N+(i-1));
    phase_type_reduce=phase_type_sampling_reduce(diff([0;phase_type_sampling_reduce])~=0);
    if length(phase_type_reduce)==1
        Pu_step=[];
        Px_step=Px_step_reduce;
    elseif length(phase_type_reduce)==2
        if phase_type_reduce(1)=='b'
            Pu_step=[];
            Px_step=Px_step_reduce(:,1:2);
        else
            Pu_step=Px_step_reduce(:,2);
            Px_step=Px_step_reduce(:,1);
        end
    else
        if phase_type_reduce(1)=='b'
            Pu_step=Px_step_reduce(:,3:end);
            Px_step=Px_step_reduce(:,1:2);
        else
            Pu_step=Px_step_reduce(:,2:end);
            Px_step=Px_step_reduce(:,1);
        end
    end
        
    H_step=Pu_step.'*Pu_step;
    
    if size(Px_step,2)==1
        xf_step=Px_step*xstep(end);
        yf_step=Px_step*ystep(end);
    else
        xf_step=Px_step*xstep(end-1:end,1);
        yf_step=Px_step*ystep(end-1:end,1);
    end

    
    %% Cost
    if isempty(Pu_step)
        xH=H_Pu+w3*H_Pu_z_mean;

        xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
            +w3*Pu_z.'*(xf_z_mean-xf_step);
        
        yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
            +w3*Pu_z.'*(yf_z_mean-yf_step);
    else
        xH=[H_Pu+w3*H_Pu_z_mean -w3*Pu_z_mean.'*Pu_step;...
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
    
    
    %% Constraints inequalities
    %TODO orientation
    %% Constraint ZMP in convex hull
    no_double_support=any(phase_type_sampling_reduce~='b',2);
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
    
    right_support=any(phase_type_sampling_reduce=='r',2);
    left_support=any(phase_type_sampling_reduce=='l',2);
    
    xb_zmp=[no_double_support(no_double_support)*fronttoankle;...
        no_double_support(no_double_support)*backtoankle;...
        no_double_support(no_double_support)*fronttoankle;...
        no_double_support(no_double_support)*backtoankle]...
        +[xf_step(no_double_support,:)-xf_z_up(no_double_support,:);...
        -xf_step(no_double_support,:)+xf_z_up(no_double_support,:);...
        xf_step(no_double_support,:)-xf_z_down(no_double_support,:);...
        -xf_step(no_double_support,:)+xf_z_down(no_double_support,:)];
    
    yb_zmp=[right_support(no_double_support)*inttoankle+left_support(no_double_support)*exttoankle;...
        right_support(no_double_support)*exttoankle+left_support(no_double_support)*inttoankle;...
        right_support(no_double_support)*inttoankle+left_support(no_double_support)*exttoankle;...
        right_support(no_double_support)*exttoankle+left_support(no_double_support)*inttoankle]...
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
        
        if phase_type_reduce(end)=='b'
            if phase_type_nodouble_reduce(end)=='r'
                phase_type_nodouble_reduce(end+1)='l';
            else
                phase_type_nodouble_reduce(end+1)='r';
            end
        end
        
        phase_type_nodouble_reduce(1)=[];
        %%%
        xb_step_stretch=[ones(size(phase_type_nodouble_reduce,1),1)*xankmax;...
            ones(size(phase_type_nodouble_reduce,1),1)*(-xankmin)];
        xb_step_stretch(1:size(Pu_step,2):end)=xb_step_stretch(1:size(Pu_step,2):end)+[xstep(end);-xstep(end)];
 
        yb_step_stretch=[any(phase_type_nodouble_reduce=='l',2)*yankmax-any(phase_type_nodouble_reduce=='r',2)*yankmin;...
            any(phase_type_nodouble_reduce=='l',2)*(-yankmin)-any(phase_type_nodouble_reduce=='r',2)*(-yankmax)];
        yb_step_stretch(1:size(Pu_step,2):end)=yb_step_stretch(1:size(Pu_step,2):end)+[ystep(end);-ystep(end)];
    end
    
    b_step_stretch=[xb_step_stretch;yb_step_stretch];
    
    % Constraint concatenation
    A=[A_zmp;A_step_stretch];
    b=[b_zmp;b_step_stretch];
    
    %% TODO Cost of com height and constraints
        %% Cost com heigth
        zf_c=Px_c*[zc(i);zdc(i);zddc(i)];

        zzmp_ref_reduce=zzmp_ref(1+(i-1):N+(i-1),:);
        hcom_ref_reduce=hcom_ref(1+(i-1):N+(i-1),:);

        zf=w4*Pu_c.'*(zf_c-zzmp_ref_reduce-hcom_ref_reduce);

        H=blkdiag(H,w4*H_c);
        f=[f;zf];
                
        %% Constraint vertical com motion
        %zeta_down (zddc - g) < (zc-zp) < zeta_up (zddc - g)
%         A_verti_motion_up=Pu_c-diag(zeta_up_ref(1+(i-1):N+(i-1),:))*Pu_ddc;
%         b_verti_motion_up=zeta_up_ref(1+(i-1):N+(i-1),:).*g+zzmp_ref_reduce-(zf_c-zeta_up_ref(1+(i-1):N+(i-1),:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);
% 
%         A_verti_motion_down=Pu_c-diag(zeta_down_ref(1+(i-1):N+(i-1),:))*Pu_ddc;
%         b_verti_motion_down=zeta_down_ref(1+(i-1):N+(i-1),:).*g+zzmp_ref_reduce-(zf_c-zeta_down_ref(1+(i-1):N+(i-1),:).*Px_ddc*[zc(i);zdc(i);zddc(i)]);
% 
%         A_verti_motion=[A_verti_motion_up;-A_verti_motion_down];
%         b_verti_motion=[b_verti_motion_up;-b_verti_motion_down];
% 
%         A=blkdiag(A,A_verti_motion);
%         b=[b;b_verti_motion];

        %% constraint kinematics com height (polyhedron)
        Pu_diff_c_p=[Pu_c zeros(N,size(Pu_step,2))]-[zeros(N,size(Pu_c,2)) Pu_step];
        z_Pu_diff_c=Pu_c;

        xf_diff_c_p=Px_c*[xc(i);xdc(i);xddc(i)]-xf_step;
        yf_diff_c_p=Px_c*[yc(i);ydc(i);yddc(i)]-yf_step;
        zf_diff_C_p=Px_c*[zc(i);zdc(i);zddc(i)]-zzmp_ref_reduce;

        A_diff_c_p_no_z=zeros(N*size(rot_successive,1),size(Pu_diff_c_p,2));
        zA_diff_c_p=zeros(N*size(rot_successive,1),size(z_Pu_diff_c,2));
        xzb_diff_c_p=zeros(N*size(rot_successive,1),1);
        yzb_diff_c_p=zeros(N*size(rot_successive,1),1);
        for j=[]%1:size(rot_successive,1)
            sin_sampled=repmat(rot_successive(j,1),N,1);
            cos_sampled=repmat(rot_successive(j,2),N,1);
            polyhedron_lim_sampled=repmat(polyhedron_lim(j,1),N,1);

            A_diff_c_p_no_z((1:N)+N*(j-1),:)=diag(sin_sampled)*Pu_diff_c_p;
            zA_diff_c_p((1:N)+N*(j-1),:)=diag(cos_sampled)*z_Pu_diff_c;

            xzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled.*xf_diff_c_p-cos_sampled.*zf_diff_C_p;
            yzb_diff_c_p((1:N)+N*(j-1),:)=polyhedron_lim_sampled-sin_sampled.*yf_diff_c_p-cos_sampled.*zf_diff_C_p;
        end

        A_diff_c_p=[A_diff_c_p_no_z zeros(size(A_diff_c_p_no_z)) zA_diff_c_p;...
            zeros(size(A_diff_c_p_no_z)) A_diff_c_p_no_z zA_diff_c_p];

        b_diff_c_p=[xzb_diff_c_p;yzb_diff_c_p];

%         A=[A;A_diff_c_p];
        A=[A zeros(size(A,1),N);A_diff_c_p];
        b=[b;b_diff_c_p];
        
    %% constraints
    Aeq=[];beq=[];lb=[];ub=[];x0=[];

    %% Options
    options=optimoptions('quadprog','Display','iter');
%     options=optimoptions('quadprog','Display','off');

    %% Optimization QP
    QP_result=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    
    xdddc_storage=[xdddc_storage;QP_result(1)];
    ydddc_storage=[ydddc_storage;QP_result(size(A_zmp,2)/2+1)];
    zdddc_storage=[zdddc_storage;QP_result(size(A_zmp,2)+1)];
    
    %% Results COM
    xc(i+1)=[1 T T^2/2]*[xc(i);xdc(i);xddc(i)]+T^3/6*xdddc_storage(end);
    xdc(i+1)=[0 1 T]*[xc(i);xdc(i);xddc(i)]+T^2/2*xdddc_storage(end);
    xddc(i+1)=[0 0 1]*[xc(i);xdc(i);xddc(i)]+T*xdddc_storage(end);
    
    yc(i+1)=[1 T T^2/2]*[yc(i);ydc(i);yddc(i)]+T^3/6*ydddc_storage(end);
    ydc(i+1)=[0 1 T]*[yc(i);ydc(i);yddc(i)]+T^2/2*ydddc_storage(end);
    yddc(i+1)=[0 0 1]*[yc(i);ydc(i);yddc(i)]+T*ydddc_storage(end);
    
    zc(i+1)=[1 T T^2/2]*[zc(i);zdc(i);zddc(i)]+T^3/6*zdddc_storage(end);
    zdc(i+1)=[0 1 T]*[zc(i);zdc(i);zddc(i)]+T^2/2*zdddc_storage(end);
    zddc(i+1)=[0 0 1]*[zc(i);zdc(i);zddc(i)]+T*zdddc_storage(end);
    
    if phase_type_sampling_reduce(1)~='b' && phase_type_sampling_reduce(2)=='b'
        xstep=[xstep;QP_result(17)];
        ystep=[ystep;QP_result(end/2+17)];
    end
    
    %% Display
    figure(1)
    clf
    title('trajectories along y')  
    hold on
    plot(1*yc(1:i+1)+0*ydc(1:i+1)-(zc(1:i+1)-zzmp_ref(1:i+1))/(zddc(1:i+1)+g).*yddc(1:i+1),'-*g')
    plot(yc(1:i+1),'-*k')
    plot((1:N)+i,Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'b') 
    
    plot(1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1:i+1).*yddc(1:i+1),'-*r')
    plot(1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1:i+1).*yddc(1:i+1),'-*m')
    plot((1:N)+i,Px_z_up*[yc(i);ydc(i);yddc(i)]+Pu_z_up*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r') 
    plot((1:N)+i,Px_z_down*[yc(i);ydc(i);yddc(i)]+Pu_z_down*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'m')
    plot((1:N)+i,(Px_z_up+Px_z_down)/2*[yc(i);ydc(i);yddc(i)]+Pu_z_mean*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'g') 
%     plot((1:N)+i,Px_z*[yc(i);ydc(i);yddc(i)]+Pu_z*QP_result(end/2+1:end/2+N),'r')
%     if i>=10
%         plot(26:26+6,repmat(-0.075,7,1),'--k')
%         plot(26:26+6,repmat(-0.2050,7,1),'--k')
%     end
    
%     if i>=18
%         plot(26+8:26+6+8,repmat(QP_result(17)-inttoankle,7,1),'--k')
%         plot(26+8:26+6+8,repmat(QP_result(17)+exttoankle,7,1),'--k')
%     end
%     
%     if i>=25
%         plot(26+16:26+6+16,repmat(QP_result(18)+inttoankle,7,1),'--k')
%         plot(26+16:26+6+16,repmat(QP_result(18)-exttoankle,7,1),'--k')
%     end
    legend('zmp','com','com preview','zmp preview','Location','southeast')
    hold off
    
    figure(2)
    clf
    title('trajectories along x')    
    hold on
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-(zc(1:i+1)-zzmp_ref(1:i+1))/(zddc(1:i+1)+g).*xddc(1:i+1),'-*g')
    plot(xc(1:i+1),'-*k')
    plot((1:N)+i,Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),'b') 
    
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-zeta_up_ref(1:i+1).*xddc(1:i+1),'-*r')
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-zeta_down_ref(1:i+1).*xddc(1:i+1),'-*m')
%     plot((1:N)+i,Px_z*[xc(i);xdc(i);xddc(i)]+Pu_z*QP_result(1:N),'r')
    legend('zmp','com','com preview','zmp preview','Location','southeast')
    hold off
    
    figure(3)
    clf
    title('trajectories along z')    
    hold on
    plot(zc(1:i+1),'-*k')
    plot((1:N)+i,Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),'b') 
    legend('com','com preview','Location','southeast')
    hold off

end
toc

% % Results ZMP
% xz=1*xc+0*xdc-h_com/g*xddc;
% yz=1*yc+0*ydc-h_com/g*yddc;
% %% Plot results
% run('script/script_plot_results.m')