clear all
clc

addpath script/ function/

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM vel ref
run('script/script_ref.m')

%% ZMP COM Linear form
run('script/script_zmp_com_linear_form.m')

%% Polyhedron limits
run('script/script_polyhedron.m')

%% Init storage QP result
run('script/script_init_storage_qp_result.m')


%% Precomputation
%same along x and y axis
w1=10^-6;
w2=10^-2;
w3=1;
w4=10^-2;

% min Jerk
H_dddc=eye(N);

% min com vel to ref
H_dc=Pu_dc.'*Pu_dc;

% min com pos to ref
H_c=Pu_c.'*Pu_c;

% Cost pre-concatenate
H_Pu=w1*H_dddc+w2*H_dc;


%% Optimization problem QP
% Sampling update
tic
for i=1:round(max(phase_duration_cumul)/T)
    i
    %% Initialization from last robot COM state
    run('script/script_initialize_from_com_state.m')
    
    %% Foot step ref
    run('script/script_initialize_from_foot_step.m')
    
    %% Cost
    run('script/script_update_cost.m')
    
    if i==120
        i
    end
    
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
    
    %% constraints
    Aeq=[];beq=[];lb=[];ub=[];x0=[];

    %% Options
    options=optimoptions('quadprog','Display','final');
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
        ystep=[ystep;QP_result(size(A_zmp,2)/2+17)];
        zstep=[zstep;zzmp_ref_reduce(3)];
    end
    %%
    if false
        %% Display
        figure(1)
        clf
        title('trajectories along y')
        xlabel('t [s]') % x-axis label
        ylabel('y [m]') % y-axis label
        hold on
        plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),'-*g')
        plot([1:i+1]*T,yc(1:i+1),'-*k')
        plot([(1:N)+i]*T,Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'b') 
    %     plot([(1:N)+i]*T,Px_z*[yc(i);ydc(i);yddc(i)]+Pu_z_mean*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r')

    %     plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-(h_com+h_com_max)/g*yddc(1:i+1),'-*r')
    %     plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-(h_com+h_com_min)/g*yddc(1:i+1),'-*m')
        plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),'-*r')
        plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),'-*m')
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

        plot([1:i+1]*T,yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),'-*b')

        hold off
        legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

        figure(2)
        clf
        title('trajectories along x')
        xlabel('t [s]') % x-axis label
        ylabel('x [m]') % y-axis label
        hold on
        plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),'-*g')
        plot([1:i+1]*T,xc(1:i+1),'-*k')
        plot(((1:N)+i)*T,Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),'b') 
    %     plot((1:N)+i,Px_z*[xc(i);xdc(i);xddc(i)]+Pu_z_mean*QP_result(1:N),'r')

        plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),'-*r')
        plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),'-*m')

    %     plot(1:i+N,0.2*(1:i+N)*T)

        plot([1:i+1]*T,xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),'-*b')

        hold off
        legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

        display=true;

        if display
            figure(3)
            clf
            title('trajectories along z')
            xlabel('t [s]') % x-axis label
            ylabel('z [m]') % y-axis label
            axis
            hold on
            plot([1:i+1]*T,zc(1:i+1),'-*k')
            plot(((1:N)+i)*T,Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),'b') 
            hold off
            legend('COM','COM preview','Location','southeast')


            figure(4)
            clf
            title('trajectory of COM on sagittal plane')
            xlabel('x [m]') % x-axis label
            ylabel('z [m]') % y-axis label
            legend('Location','southeast')
            hold on
            plot(xc(1:i+1),zc(1:i+1),'-*k')
            hold off

            figure(5)
            clf
            title('trajectories of COM on lateral plane')
            xlabel('y [m]') % x-axis label
            ylabel('z [m]') % y-axis label
            legend('Location','southeast')
            hold on
            plot(yc(1:i+1),zc(1:i+1),'-*k')
            hold off
        end


        figure(6)
        clf
        title('trajectories along y')
        xlabel('x [m]') % x-axis label
        ylabel('y [m]') % y-axis label
        hold on
        plot(1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),'-*g')
        plot(xc(1:i+1),yc(1:i+1),'-*k')
        plot(Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'b') 

        plot(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),'-*r')
        plot(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),'-*m')

        plot(xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),'-*b')

        hold off
        legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

        hold on;
        pstep=[xstep ystep];
        psi=zeros(size(pstep,1)*3,1);
        firstSS=(phase_type(2)=='r');

        XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
        for j=1:length(pstep)
            plot(XY(j,1:5),XY(j,6:10),'-k','LineWidth',2)
        end

        XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
        for j=1:length(pstep)
            plot(XY(j,1:5),XY(j,6:10),':k','LineWidth',2)
        end
        hold off
%     figure(3)
%             clf
%             title('trajectories along z')
%             xlabel('t [s]') % x-axis label
%             ylabel('z [m]') % y-axis label
%             hold on
%             plot([1:i+1]*T,zc(1:i+1),'-*k')
%             plot(((1:N)+i)*T,Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),'b') 
%             hold off
%             legend('COM','COM preview','Location','southeast')
%             
%             figure(4)
%             clf
%             hold on
%             plot(zc(1:i+1)./(zddc(1:i+1)+g))
%             plot(zeta_up_ref,'r')
%             plot(zeta_down_ref,'m')
%             hold off
        figure(6)
        clf
        title('trajectories along y')
        xlabel('x [m]') % x-axis label
        ylabel('y [m]') % y-axis label
        zlabel('z [m]') % z-axis label
        view(3)
        axis([-0.5 4 -2.25 2.25 -inf 1])
        hold on
        plot3(1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),...
            1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),...
            [zzmp_ref(1);zzmp_ref(1:i)],...
            '-*g')
        plot3(xc(1:i+1),yc(1:i+1),zc(1:i+1),'-*k')
        plot3(Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),...
            Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),...
            Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),...
            'b') 

        plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),...
            1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),...
            [zzmp_ref(1);zzmp_ref(1:i)],...
            '-*r')
        plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),...
            1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),...
            [zzmp_ref(1);zzmp_ref(1:i)],...
            '-*m')

        plot3(xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),...
            yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),...
            [zzmp_ref(1);zzmp_ref(1:i)],...
            '-*b')

        hold off
        legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

        hold on;
        pstep=[xstep ystep];
        psi=zeros(size(pstep,1)*3,1);
        firstSS=(phase_type(2)=='r');

        XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
        for j=1:length(pstep)
            plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
        end

        XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
        for j=1:length(pstep)
            plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
        end
        
        XY=drawing_rectangle_rotate(pstep,psi,backtoankle+sole_margin,fronttoankle+sole_margin,exttoankle+sole_margin,inttoankle+sole_margin,firstSS);
        for j=1:length(pstep)
            fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
        end
        hold off
    end
        
%         figure(7)
%         clf
% 
%         ax1 = subplot(2,1,1);
%         title('trajectories 3d')
%         xlabel('x [m]') % x-axis label
%         ylabel('y [m]') % y-axis label
%         zlabel('z [m]') % z-axis label
%         view(3)
%         hold on
%         plot3(xc(1:i+1),yc(1:i+1),zc(1:i+1),'-*k')
%         plot3(Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),...
%             Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),...
%             Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),...
%             'b')
%         hold off
% 
%         ax2 = subplot(2,1,2);
%         title('trajectories 3d')
%         xlabel('x [m]') % x-axis label
%         ylabel('y [m]') % y-axis label
%         zlabel('z [m]') % z-axis label
%         view(3)
%         hold on
%         plot3(1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),...
%             1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),...
%             [zzmp_ref(1);zzmp_ref(1:i)],...
%             '-*g')
%         
%         plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),...
%             1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),...
%             [zzmp_ref(1);zzmp_ref(1:i)],...
%             '-*r')
%         plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),...
%             1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),...
%             [zzmp_ref(1);zzmp_ref(1:i)],...
%             '-*m')
% 
%         plot3(xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),...
%             yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),...
%             [zzmp_ref(1);zzmp_ref(1:i)],...
%             '-*b')
%         hold off
%         hold on;
%         pstep=[xstep ystep];
%         psi=zeros(size(pstep,1)*3,1);
%         firstSS=(phase_type(2)=='r');
% 
%         XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
%         for j=1:length(pstep)
%             plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
%         end
% 
%         XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
%         for j=1:length(pstep)
%             plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
%         end
%         
%         XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
%         for j=1:length(pstep)
%             fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
%         end
%         hold off
%         hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'}); 
%         rotate3d on
%         
%         legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')
        
end
toc

%% Results ZMP
zz=[zzmp_ref(1);zzmp_ref(1:end-1)];
zz(length(zc)+1:end)=[];
xz=1*xc+0*xdc-(zc-zz)./(zddc+g).*xddc;
yz=1*yc+0*ydc-(zc-zz)./(zddc+g).*yddc;

zz_up=zz;
xz_up=1*xc+0*xdc-zeta_up_ref(1:length(zc)).*xddc;
yz_up=1*yc+0*ydc-zeta_up_ref(1:length(zc)).*yddc;

zz_down=zz;
xz_down=1*xc+0*xdc-zeta_down_ref(1:length(zc)).*xddc;
yz_down=1*yc+0*ydc-zeta_down_ref(1:length(zc)).*yddc;

zcapture=zz;
xcapture=xc+((zc-zz)./(zddc+g)).^(1/2).*xdc;
ycapture=yc+((zc-zz)./(zddc+g)).^(1/2).*ydc;


% %% Plot results
% run('script/script_plot_results.m')

figure(7)
clf

ax1 = subplot(2,1,1);
title('trajectory of COM 3d')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
view(3)
hold on
plot3(xc,yc,zc,'-*k')
hold off
legend('COM','Location','southeast')

ax2 = subplot(2,1,2);
title('trajectories 3d')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
view(3)
hold on
plot3(xz,...
    yz,...
    zz,...
    '-*g')

plot3(xz_up,...
    yz_up,...
    zz_up,...
    '-*r')
plot3(xz_down,...
    yz_down,...
    zz_down,...
    '-*m')

plot3(xcapture,...
    ycapture,...
    zcapture,...
    '-*b')
hold off
hold on;
pstep=[xstep ystep];
psi=zeros(size(pstep,1)*3,1);
firstSS=(phase_type(2)=='r');

XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
for j=1:length(pstep)
    plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
for j=1:length(pstep)
    plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
for j=1:length(pstep)
    fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
end
hold off
hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'}); 
rotate3d on

legend('CoP','CoP up','CoP down','Capture point','Location','southeast')

