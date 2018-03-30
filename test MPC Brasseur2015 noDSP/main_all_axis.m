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
    
    %% Constraint Inequalities
    run('script/script_cons_ineq.m')
    
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


%% Plot results
run('script/script_plot_results.m')

%%
hstep_move=0.05;
pstep_3d=[pstep zstep];
pstep_3d_phase_r=[];
pstep_3d_phase_l=[];
for i=1:length(phase_type)
    if phase_type(i)=='b'
        if i<length(phase_type)
            if phase_type(i+1)=='r'
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
            else
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
            end
        else
            if phase_type(i-1)=='l'
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
            else
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
            end
        end
    pstep_3d(1,:)=[];
    elseif phase_type(i)=='r'
        pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);pstep_3d(1,:);];
        pstep_3d_phase_l=[pstep_3d_phase_l;...
            [(pstep_3d_phase_l(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_l(end,3),pstep_3d(2,3))+hstep_move];...
            pstep_3d(2,:);];
    else
        pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);pstep_3d(1,:);];
        pstep_3d_phase_r=[pstep_3d_phase_r;...
            [(pstep_3d_phase_r(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_r(end,3),pstep_3d(2,3))+hstep_move];...
             pstep_3d(2,:);];
    end
end
%%
% figure(8)
% clf
% title('trajectories foot in air')
% 
% ax1 = subplot(3,1,1);
% ylabel('x [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,1))
% plot(pstep_3d_phase_l(:,1))
% hold off
% 
% ax2 = subplot(3,1,2);
% ylabel('y [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,2))
% plot(pstep_3d_phase_l(:,2))
% hold off
% 
% ax3 = subplot(3,1,3);
% ylabel('z [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,3))
% plot(pstep_3d_phase_l(:,3))
% hold off
% linkaxes([ax1,ax2,ax3],'x')
% legend('R foot','Lfoot','Location','southeast')

%%
phase_type_enlarge=ones(length(phase_type) + length(phase_type(2:2:end)),1);
phase_type_enlarge(:) = nan;
phase_type_enlarge(3:3:end,:)=phase_type(2:2:end,:);
locations = any(isnan(phase_type_enlarge),2);
phase_type_enlarge(locations) = phase_type;
phase_type_enlarge=char(phase_type_enlarge);

phase_duration_enlarge=zeros(length(phase_type_enlarge),1);
phase_duration_enlarge(any(phase_type_enlarge=='r',2))=phase_duration_r/2;
phase_duration_enlarge(any(phase_type_enlarge=='l',2))=phase_duration_l/2;
phase_duration_enlarge(any(phase_type_enlarge=='b',2))=phase_duration_b;
phase_duration_enlarge(1)=phase_duration_start;
phase_duration_enlarge(end)=phase_duration_stop;

phase_duration_cumul_enlarge=zeros(length(phase_duration_enlarge),1);
for i=1:length(phase_duration_enlarge)
    phase_duration_cumul_enlarge(i,1)=sum(phase_duration_enlarge(1:i,1));
end

[A_f,B_f]=compute_coeff_poly6_interpolation([0;phase_duration_cumul_enlarge],[],[],size(phase_type_enlarge,1),(size(phase_type_enlarge,1)+1)*3);

%%
%   1. Define new vector Y based on existing vector x,
viapoint_l = ones(length(pstep_3d_phase_l)+1 + length(pstep_3d_phase_l)*2+2,3);
viapoint_l(:) = nan;
%2. Insert New.Values into respective cells of Y, for example
viapoint_l(2:3:end,:) = 0;
viapoint_l(3:3:end,:) = 0;
% 3. after all new values inserted, find locations of NaN in Y;
locations = find(isnan(viapoint_l));
% 4. Replace the found locations of Y with existing x values;
viapoint_l(locations) = [pstep_3d_phase_l(1,:);pstep_3d_phase_l;];

%
A_f_l=A_f*viapoint_l;

%   1. Define new vector Y based on existing vector x,
viapoint_r = ones(length(pstep_3d_phase_r)+1 + length(pstep_3d_phase_r)*2+2,3);
viapoint_r(:) = nan;
%2. Insert New.Values into respective cells of Y, for example
viapoint_r(2:3:end,:) = 0;
viapoint_r(3:3:end,:) = 0;
% 3. after all new values inserted, find locations of NaN in Y;
locations = find(isnan(viapoint_r));
% 4. Replace the found locations of Y with existing x values;
viapoint_r(locations) = [pstep_3d_phase_r(1,:);pstep_3d_phase_r;];

%
A_f_r=A_f*viapoint_r;
%%
% dt_time=zeros(length(phase_type_enlarge),size(A_f_l,1));
% for i=1:size(dt_time,1)
%     dt_time(i,(i-1)*6+1)=1;
% end
% 
% toto=dt_time*A_f_l;

frequency=200;
discretization=phase_duration_enlarge*200;

phase_type_sampling_enlarge=[];
for i=1:length(phase_duration_enlarge)
    phase_type_sampling_enlarge=[phase_type_sampling_enlarge;repmat(phase_type_enlarge(i),discretization(i),1)];
end

[mdt] = compute_coeff_dt_matrix(discretization,frequency,length(phase_type_enlarge),length(phase_type_sampling_enlarge)+1);
[mddt] = compute_coeff_ddt_matrix(discretization,frequency);
[mdddt] = compute_coeff_dddt_matrix(discretization,frequency);

pankle_l=mdt*A_f_l;
pankle_r=mdt*A_f_r;

vankle_l=mddt*A_f_l;
vankle_r=mddt*A_f_r;

aankle_l=mdddt*A_f_l;
aankle_r=mdddt*A_f_r;

%%
figure(9)
clf
title('trajectories foot in air')

ax1 = subplot(3,1,1);
ylabel('x [m]') % y-axis label
hold on
plot(pankle_r(:,1))
plot(pankle_l(:,1))
hold off

ax2 = subplot(3,1,2);
ylabel('y [m]') % y-axis label
hold on
plot(pankle_r(:,2))
plot(pankle_l(:,2))
hold off

ax3 = subplot(3,1,3);
ylabel('z [m]') % y-axis label
hold on
plot(pankle_r(:,3))
plot(pankle_l(:,3))
hold off
linkaxes([ax1,ax2,ax3],'x')
legend('R foot','Lfoot','Location','southeast')

%% 
t=[[1/frequency:1/frequency:T]' [(1/frequency:1/frequency:T).^2./2]'];
t=[ones(size(t,1),1) t];
dt=[zeros(size(t,1),1) ones(size(t,1),1) [1/frequency:1/frequency:T]'];
ddt=[zeros(size(t,1),1) zeros(size(t,1),1) ones(size(t,1),1)];
dddt=[(1/frequency:1/frequency:T)]';

xc_discret=[xc(1)];
xdc_discret=[xc(1)];
xddc_discret=[xc(1)];

yc_discret=[yc(1)];
ydc_discret=[yc(1)];
yddc_discret=[yc(1)];

zc_discret=[zc(1)];
zdc_discret=[zc(1)];
zddc_discret=[zc(1)];
for i=1:size(xdddc_storage,1)
    xc_discret=[xc_discret;t*[xc(i);xdc(i);xddc(i)]+dddt.^3/6*xdddc_storage(i)];
    xdc_discret=[xdc_discret;dt*[xc(i);xdc(i);xddc(i)]+dddt.^2/2*xdddc_storage(i)];
    xddc_discret=[xddc_discret;ddt*[xc(i);xdc(i);xddc(i)]+dddt*xdddc_storage(i)];
    
    yc_discret=[yc_discret;t*[yc(i);ydc(i);yddc(i)]+dddt.^3/6*ydddc_storage(i)];
    ydc_discret=[ydc_discret;dt*[yc(i);ydc(i);yddc(i)]+dddt.^2/2*ydddc_storage(i)];
    yddc_discret=[yddc_discret;ddt*[yc(i);ydc(i);yddc(i)]+dddt*ydddc_storage(i)];
    
    zc_discret=[zc_discret;t*[zc(i);zdc(i);zddc(i)]+dddt.^3/6*zdddc_storage(i)];
    zdc_discret=[zdc_discret;dt*[zc(i);zdc(i);zddc(i)]+dddt.^2/2*zdddc_storage(i)];
    zddc_discret=[zddc_discret;ddt*[zc(i);zdc(i);zddc(i)]+dddt*zdddc_storage(i)];
end

zzmp_ref_discret=zeros(round(max(phase_duration_cumul)*frequency),1);
zzmp_ref_discret(round(phase_duration_cumul(3)*frequency)+1:round(phase_duration_cumul(5)*frequency))=0.05;
zzmp_ref_discret(round(phase_duration_cumul(5)*frequency)+1:round(phase_duration_cumul(7)*frequency))=0.1;
zzmp_ref_discret(round(phase_duration_cumul(7)*frequency)+1:round(phase_duration_cumul(9)*frequency))=0.15;
zzmp_ref_discret(round(phase_duration_cumul(9)*frequency)+1:end)=0.2;
% zz=[zzmp_ref(1);zzmp_ref(1:end-1)];
% zz(length(zc)+1:end)=[];
zz_discret=[zzmp_ref_discret(1);zzmp_ref_discret(1:end)];

xz_discret=1*xc_discret+0*xdc_discret-(zc_discret-zz_discret)./(zddc_discret+g).*xddc_discret;
yz_discret=1*yc_discret+0*ydc_discret-(zc_discret-zz_discret)./(zddc_discret+g).*yddc_discret;
%%
figure()
clf
hold on
plot(xz,yz,'o')
plot(xz_discret,yz_discret)
hold off

figure()
clf
hold on
plot(xc,yc,'o')
plot(xc_discret,yc_discret)
hold off

%% 
dt_type_phase_=any(phase_type_sampling_enlarge=='l',2)*1+any(phase_type_sampling_enlarge=='r',2)*2;
dt_type_phase_=[0;dt_type_phase_];
    
%% write txt
e_=0.099;
e=0;
trajectories=[dt_type_phase_ ...
        xz_discret yz_discret zz_discret ...
        xc_discret yc_discret zc_discret ...
        xdc_discret ydc_discret zdc_discret ...
        xddc_discret yddc_discret zddc_discret ...
        pankle_l(:,1) pankle_l(:,2) pankle_l(:,3)+e_-e ...
        vankle_l(:,1) vankle_l(:,2) vankle_l(:,3) ...
        aankle_l(:,1) aankle_l(:,2) aankle_l(:,3) ...
        pankle_r(:,1) pankle_r(:,2) pankle_r(:,3)+e_-e ...
        vankle_r(:,1) vankle_r(:,2) vankle_r(:,3) ...
        aankle_r(:,1) aankle_r(:,2) aankle_r(:,3) ...
        pankle_l(:,1)*0 pankle_l(:,2)*0 pankle_l(:,3)*0 ... %rtheta_dt_r rphi_dt_r
        pankle_r(:,1)*0 pankle_r(:,2)*0 pankle_r(:,3)*0]; %rtheta_dt_r rphi_dt_r
    
%% %save data in txt
% zmpcom=fopen('zmp_com_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');
zmpcom=fopen('zmp_com_test.txt','w');


for i=1:size(trajectories,1)
    fprintf(zmpcom,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',trajectories(i,:));
end
fclose(zmpcom);

%%
psi_=pstep(:,1)*0;

pstep_=[pstep(:,1) pstep(:,2) psi_];
% pstepf=fopen('pstep_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');
pstepf=fopen('pstep_test.txt','w');


for i=1:size(pstep_,1)
    fprintf(pstepf,'%f %f %f\n',pstep_(i,:));
end
fclose(pstepf);

fclose('all');
%%%%%%%%%%%