clear all
clc

addpath script/ function/

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM vel ref
run('script/script_com_vel_ref.m')

%% ZMP COM Linear form
run('script/script_zmp_com_linear_form.m')

%% Init storage QP result
run('script/script_init_storage_qp_result.m')


%% Precomputation
%same along x and y axis
w1=10^-6;
w2=10^-2;
w3=1;

% min Jerk
H_dddc=eye(N);

% min com vel to ref
H_dc=Pu_dc.'*Pu_dc;

% min zmp to ref
H_z=Pu_z.'*Pu_z;

H_Pu=w1*H_dddc+w2*H_dc+w3*H_z;


%% Optimization problem QP
% Sampling update
tic
for i=1:round(max(phase_duration_cumul)/T)
    i
    %% COM velocity
    xf_dc=Px_dc*[xc(i);xdc(i);xddc(i)];
    yf_dc=Px_dc*[yc(i);ydc(i);yddc(i)];
    
    %% COM velocity ref
    xf_dc_ref=xvcom_ref(1+(i-1):N+(i-1));
    yf_dc_ref=yvcom_ref(1+(i-1):N+(i-1));
    
    
    %% ZMP
    xf_z=Px_z*[xc(i);xdc(i);xddc(i)];
    yf_z=Px_z*[yc(i);ydc(i);yddc(i)];
    
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
        xH=H_Pu;

        xf=w2*Pu_dc.'*(xf_dc-xf_dc_ref)...
            +w3*Pu_z.'*(xf_z-xf_step);
        
        yf=w2*Pu_dc.'*(yf_dc-yf_dc_ref)...
            +w3*Pu_z.'*(yf_z-yf_step);
    else
        xH=[H_Pu -w3*Pu_z.'*Pu_step;...
            (-w3*Pu_z.'*Pu_step).' w3*H_step];
        
        xf=[w2*Pu_dc.'*(xf_dc-xf_dc_ref)+...
            w3*Pu_z.'*(xf_z-xf_step);...
            -w3*Pu_step.'*(xf_z-xf_step)];
        
        yf=[w2*Pu_dc.'*(yf_dc-yf_dc_ref)+...
            w3*Pu_z.'*(yf_z-yf_step);...
            -w3*Pu_step.'*(yf_z-yf_step)];
    end
    
    yH=xH;
    
    H=blkdiag(xH,yH);
    
    f=[xf;yf];
    
    
    %% Constraints inequalities
    %TODO orientation
    % Constraint ZMP in convex hull
    no_double_support=any(phase_type_sampling_reduce~='b',2);
    if isempty(Pu_step)
        A_zmp=[Pu_z(no_double_support,:);...
            -Pu_z(no_double_support,:)];
    else
        A_zmp=[Pu_z(no_double_support,:) -Pu_step(no_double_support,:);...
            -Pu_z(no_double_support,:) Pu_step(no_double_support,:)];
    end
    
    A_zmp=blkdiag(A_zmp,A_zmp);
    
    right_support=any(phase_type_sampling_reduce=='r',2);
    left_support=any(phase_type_sampling_reduce=='l',2);
    
    xb_zmp=[no_double_support(no_double_support)*fronttoankle;...
        no_double_support(no_double_support)*backtoankle]...
        +[xf_step(no_double_support,:)-xf_z(no_double_support,:);...
        -xf_step(no_double_support,:)+xf_z(no_double_support,:)];
    
    yb_zmp=[right_support(no_double_support)*inttoankle+left_support(no_double_support)*exttoankle;...
        right_support(no_double_support)*exttoankle+left_support(no_double_support)*inttoankle]...
        +[yf_step(no_double_support,:)-yf_z(no_double_support,:);...
        -yf_step(no_double_support,:)+yf_z(no_double_support,:)];
    
    b_zmp=[xb_zmp;yb_zmp];
    
    % Constraint foot step stretching
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
    %% constraints
    Aeq=[];beq=[];lb=[];ub=[];x0=[];

    %% Options
    options=optimoptions('quadprog','Display','iter');
%     options=optimoptions('quadprog','Display','off');

    %% Optimization QP
    QP_result=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    
    xdddc_storage=[xdddc_storage;QP_result(1)];
    ydddc_storage=[ydddc_storage;QP_result(end/2+1)];
    
    %% Results COM
    xc(i+1)=[1 T T^2/2]*[xc(i);xdc(i);xddc(i)]+T^3/6*xdddc_storage(end);
    xdc(i+1)=[0 1 T]*[xc(i);xdc(i);xddc(i)]+T^2/2*xdddc_storage(end);
    xddc(i+1)=[0 0 1]*[xc(i);xdc(i);xddc(i)]+T*xdddc_storage(end);
    
    yc(i+1)=[1 T T^2/2]*[yc(i);ydc(i);yddc(i)]+T^3/6*ydddc_storage(end);
    ydc(i+1)=[0 1 T]*[yc(i);ydc(i);yddc(i)]+T^2/2*ydddc_storage(end);
    yddc(i+1)=[0 0 1]*[yc(i);ydc(i);yddc(i)]+T*ydddc_storage(end);
    
    if phase_type_sampling_reduce(1)~='b' && phase_type_sampling_reduce(2)=='b'
        xstep=[xstep;QP_result(17)];
        ystep=[ystep;QP_result(end/2+17)];
    end
    
    %% Display
    figure(1)
    clf
    title('trajectories along y')
    legend('zmp','com','com preview','zmp preview','Location','southeast')
    hold on
    plot(1*yc(1:i+1)+0*ydc(1:i+1)-h_com/g*yddc(1:i+1),'-*g')
    plot(yc(1:i+1),'-*k')
    plot((1:N)+i,Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(end/2+1:end/2+N),'b') 
    plot((1:N)+i,Px_z*[yc(i);ydc(i);yddc(i)]+Pu_z*QP_result(end/2+1:end/2+N),'r')
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
    hold off
    
    figure(2)
    clf
    title('trajectories along x')
    legend('zmp','com','com preview','zmp preview','Location','southeast')
    hold on
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-h_com/g*xddc(1:i+1),'-*g')
    plot(xc(1:i+1),'-*k')
    plot((1:N)+i,Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),'b') 
    plot((1:N)+i,Px_z*[xc(i);xdc(i);xddc(i)]+Pu_z*QP_result(1:N),'r')
    
    plot(1:i+N,0.2*(1:i+N)*T)
    hold off

end
toc

% % Results ZMP
% xz=1*xc+0*xdc-h_com/g*xddc;
% yz=1*yc+0*ydc-h_com/g*yddc;
% %% Plot results
% run('script/script_plot_results.m')