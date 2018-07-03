clear all

clc

addpath script/ function/

walking_type=1;
% 1 : walking flat
% 2 : walking airbus stairs
% 3 : walking flat quick

save_figure=false;

%% Constant
run('script/script_constant.m')

%% Phase duration
run('script/script_phase_duration.m')

%% COM vel ref
run('script/script_ref.m')

%% ZMP COM Linear form
COM_form='com jerk'
%'com jerk' : COM with piece-wise jerk
%'zmp vel' : ZMP with piece-wise velocity
switch(COM_form)
    case 'com jerk'
        run('script/script_zmp_com_linear_form_comJerk.m')
    case 'zmp vel'
end

%% Polyhedron limits
kinematic_limit='hexagonTranslation';
%'' : very simple polyhedron
%'hexagon' : hexagon kinematic limits
switch kinematic_limit
    case ''
        number_level=[];
        run('script/script_polyhedron.m')
    case 'hexagon'
        number_level=2;
        run('script/script_polyhedron_hexagon.m')
    case 'hexagonTranslation'
        number_level=2;
        run('script/script_polyhedron_hexagon_translation.m')
    otherwise
        error('choose a type of kinematic_limit')
end

%% Init storage QP result
run('script/script_init_storage_qp_result.m')


%% Precomputation
%same along x and y axis
% w1=10^-6; %jerk
% w2=10^-2; %com vel ref
% w3=10^-2; %zmp wth zeta mean close to step
% w4=10^-1; %com height

%as Camille
w1=10^-7; %jerk
w2=10^0; %com vel ref
w3=10^-1; %zmp wth zeta mean close to step
w4=10^-2; %com height

% w1=10^-5; %jerk
% w2=10^0; %com vel ref
% w3=10^0; %zmp wth zeta mean close to step
% w4=10^0; %com height

% w1=1.5*10^-4; %jerk
% w2=10^0; %com vel ref
% w3=10^0; %zmp wth zeta mean close to step
% w4=10^0; %com height

switch(COM_form)
    case 'com jerk'
        % min Jerk
        H_dddc=eye(N);

        % min com vel to ref
        H_dc=Pu_dc.'*Pu_dc;

        % min com pos to ref
        H_c=Pu_c.'*Pu_c;

        % Cost pre-concatenate
        H_Pu=w1*H_dddc+w2*H_dc;
    case 'zmp vel'
        H_dz=eye(N);
end


converge_sampling=[];
%% Optimization problem QP
% Sampling update
tic
for i=1:round(max(phase_duration_cumul)/T)
    i
    %% Initialization from last robot COM state
    switch(COM_form)
        case 'com jerk'
            run('script/script_initialize_from_com_state_comJerk.m')
        case 'zmp vel'
            run('script/script_initialize_from_com_state_zmpVel.m')
    end
    
    
    %% Foot step ref
    run('script/script_initialize_from_foot_step.m')
    
    %% Cost
    switch(COM_form)
        case 'com jerk'
            run('script/script_update_cost_comJerk.m')
        case 'zmp vel'
            run('script/script_update_cost_zmpVel.m')
    end
    
    
    %% Constraint Inequalities
    switch(COM_form)
        case 'com jerk'
            run('script/script_cons_ineq_comJerk.m')
        case 'zmp vel'
            run('script/script_cons_ineq_zmpVel.m')
    end
    
    
    %% Constraint Equalities
    switch(COM_form)
        case 'com jerk'
            run('script/script_cons_eq_comJerk.m')
        case 'zmp vel'
            run('script/script_cons_eq_zmpVel.m')
    end
    
    
    %% constraints
    lb=[];ub=[];x0=[];

    %% Options
%     options=optimoptions('quadprog','Display','iter','MaxIterations',1000);
    options=optimoptions('quadprog','Display','final');
%     options=optimoptions('quadprog','Display','off');

    %% Optimization QP
    [QP_result,tata,converge_sampling(i)]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);

    switch(COM_form)
        case 'com jerk'
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


            %%
            if false
                run('script/script_display_online.m')
            end 
        case 'zmp vel'
            xdddc_storage=[xdddc_storage;QP_result(1)];
            ydddc_storage=[ydddc_storage;QP_result(size(A_zmp,2)/2+1)];
            zdddc_storage=[zdddc_storage;QP_result(size(A_zmp,2)+1)];
            
            %%
            xc(i+1)=xf_c(1,:)+Pu_c(1,1)*xdddc_storage(end);
            xdc(i+1)=xf_dc(1,:)+Pu_dc(1,1)*xdddc_storage(end);
            xddc(i+1)=xf_ddc(1,:)+Pu_ddc(1,1)*xdddc_storage(end);

            yc(i+1)=yf_c(1,:)+Pu_c(1,1)*ydddc_storage(end);
            ydc(i+1)=yf_dc(1,:)+Pu_dc(1,1)*ydddc_storage(end);
            yddc(i+1)=yf_ddc(1,:)+Pu_ddc(1,1)*ydddc_storage(end);

            zc(i+1)=zf_c(1,:)+Pu_c(1,1)*zdddc_storage(end);
            zdc(i+1)=zf_dc(1,:)+Pu_dc(1,1)*zdddc_storage(end);
            zddc(i+1)=zf_ddc(1,:)+Pu_ddc(1,1)*zdddc_storage(end);

    end    
    
    if phase_type_sampling_reduce(1)~='b' && phase_type_sampling_reduce(2)=='b'
    xstep=[xstep;QP_result(17)];
    ystep=[ystep;QP_result(size(A_zmp,2)/2+17)];
    zstep=[zstep;zzmp_ref_reduce(3)];
    end      

    figure(1)
    clf
    hold on
    plot([0:i]*T,xc(1:i+1),'b')
    plot([i:i+15]*T,xf_c+Pu_c*QP_result(1:16),'r')
    plot([0:i]*T,yc(1:i+1),'b')
    plot([i:i+15]*T,yf_c+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+16),'r')
    plot([0:i]*T,zc(1:i+1),'b')
    plot([i:i+15]*T,zf_c+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+16),'r')
    hold off
end
toc

%% Results ZMP
zz=[zzmp_ref(1);zzmp_ref(1:end-1)];
zz(length(zc)+1:end)=[];
xz=1*xc+0*xdc-(zc-zz)./(zddc+g).*xddc;
yz=1*yc+0*ydc-(zc-zz)./(zddc+g).*yddc;

zz_up=zz;
xz_up=[xz(1);1*xc(2:end)+0*xdc(2:end)-zeta_up_ref(1:length(zc)-1).*xddc(2:end)];
yz_up=[yz(1);1*yc(2:end)+0*ydc(2:end)-zeta_up_ref(1:length(zc)-1).*yddc(2:end)];

zz_down=zz;
xz_down=1*xc+0*xdc-zeta_down_ref(1:length(zc)).*xddc;
yz_down=1*yc+0*ydc-zeta_down_ref(1:length(zc)).*yddc;

zcapture=zz;
xcapture=xc+((zc-zz)./(zddc+g)).^(1/2).*xdc;
ycapture=yc+((zc-zz)./(zddc+g)).^(1/2).*ydc;

xdcm=xc+((zc-zz)./(zddc+g)).^(1/2).*xdc;
ydcm=yc+((zc-zz)./(zddc+g)).^(1/2).*ydc;
zdcm=zc+((zc-zz)./(zddc+g)).^(1/2).*zdc;

zeta=(zc-zz)./(zddc+g);

%%
pstep=[xstep ystep];
psi=zeros(size(pstep,1)*3,1);
firstSS=(phase_type(2)=='r');
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

% %%
% figure(9)
% clf
% title('trajectories foot in air')
% 
% ax1 = subplot(3,1,1);
% ylabel('x [m]') % y-axis label
% hold on
% plot(pankle_r(:,1))
% plot(pankle_l(:,1))
% hold off
% 
% ax2 = subplot(3,1,2);
% ylabel('y [m]') % y-axis label
% hold on
% plot(pankle_r(:,2))
% plot(pankle_l(:,2))
% hold off
% 
% ax3 = subplot(3,1,3);
% ylabel('z [m]') % y-axis label
% hold on
% plot(pankle_r(:,3))
% plot(pankle_l(:,3))
% hold off
% linkaxes([ax1,ax2,ax3],'x')
% legend('R foot','Lfoot','Location','southeast')

%% 

switch(COM_form)
        case 'com jerk'
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
        case 'zmp vel'
            t=[sinh(omega_temp*[1/frequency:1/frequency:T]')/omega_temp (cosh(omega_temp*[1/frequency:1/frequency:T]')-1)/omega_temp^2];
            t=[ones(size(t,1),1) t];            
            dt=[zeros(size(t,1),1) cosh(omega_temp*[1/frequency:1/frequency:T]') sinh(omega_temp*[1/frequency:1/frequency:T]')/omega_temp];
            ddt=[zeros(size(t,1),1) omega_temp*sinh(omega_temp*[1/frequency:1/frequency:T]') cosh(omega_temp*[1/frequency:1/frequency:T]')];
            dddt=[-sinh(omega_temp*[1/frequency:1/frequency:T]')/omega_temp+[1/frequency:1/frequency:T]' -cosh(omega_temp*[1/frequency:1/frequency:T]')+1 -omega_temp*sinh(omega_temp*[1/frequency:1/frequency:T]')];

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
                xc_discret=[xc_discret;t*[xc(i);xdc(i);xddc(i)]+dddt(:,1)*xdddc_storage(i)];
                xdc_discret=[xdc_discret;dt*[xc(i);xdc(i);xddc(i)]+dddt(:,2)*xdddc_storage(i)];
                xddc_discret=[xddc_discret;ddt*[xc(i);xdc(i);xddc(i)]+dddt(:,3)*xdddc_storage(i)];

                yc_discret=[yc_discret;t*[yc(i);ydc(i);yddc(i)]+dddt(:,1)*ydddc_storage(i)];
                ydc_discret=[ydc_discret;dt*[yc(i);ydc(i);yddc(i)]+dddt(:,2)*ydddc_storage(i)];
                yddc_discret=[yddc_discret;ddt*[yc(i);ydc(i);yddc(i)]+dddt(:,3)*ydddc_storage(i)];

                zc_discret=[zc_discret;t*[zc(i);zdc(i);zddc(i)]+dddt(:,1)*zdddc_storage(i)];
                zdc_discret=[zdc_discret;dt*[zc(i);zdc(i);zddc(i)]+dddt(:,2)*zdddc_storage(i)];
                zddc_discret=[zddc_discret;ddt*[zc(i);zdc(i);zddc(i)]+dddt(:,3)*zdddc_storage(i)];
            end
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
% %%
% figure(1)
% clf
% hold on
% plot(xz,yz,'o')
% plot(xz_discret,yz_discret)
% hold off
% 
% figure(2)
% clf
% hold on
% plot(xc,yc,'o')
% plot(xc_discret,yc_discret)
% hold off

%% 
dt_type_phase_=any(phase_type_sampling_enlarge=='l',2)*1+any(phase_type_sampling_enlarge=='r',2)*2;
dt_type_phase_=[0;dt_type_phase_];

%% Plot results
run('script/script_plot_results.m')
    
%% write txt
if false
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
end
%%
if save_figure
    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D'])
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta'])
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal'])
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal'])
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView'])

    saveas(figure(8),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_3D']...
        ,'png')
    saveas(figure(10),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_zeta']...
        ,'png')
    saveas(figure(11),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMfrontal']...
        ,'png')
    saveas(figure(12),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_COMsagittal' ]...
        ,'png')
    saveas(figure(13),['save_figure/figure_type_' num2str(walking_type) ...
        '_w1_' num2str(w1,'%10.0e') '_w2_' num2str(w2,'%10.0e') '_w3_' num2str(w3,'%10.0e') '_w4_' num2str(w4,'%10.0e') ...
        '_' kinematic_limit num2str(number_level) '_TopView' ]...
        ,'png')
end