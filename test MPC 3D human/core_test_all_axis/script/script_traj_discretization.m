%% discretization trajectories
switch(COM_form)
        case 'comPolynomial'
            xc_discret=[MPC_outputs_storage.xc(1)];
            xdc_discret=[MPC_outputs_storage.xdc(1)];
            xddc_discret=[MPC_outputs_storage.xddc(1)];

            yc_discret=[MPC_outputs_storage.yc(1)];
            ydc_discret=[MPC_outputs_storage.ydc(1)];
            yddc_discret=[MPC_outputs_storage.yddc(1)];

            zc_discret=[MPC_outputs_storage.zc(1)];
            zdc_discret=[MPC_outputs_storage.zdc(1)];
            zddc_discret=[MPC_outputs_storage.zddc(1)];
            for i=1:size(MPC_outputs_storage.xc_control,1)
                t=[[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]' [(1/frequency:1/frequency:experiment.phase_duration_sampling(i)).^2./2]'];
                t=[ones(size(t,1),1) t];
                dt=[zeros(size(t,1),1) ones(size(t,1),1) [1/frequency:1/frequency:experiment.phase_duration_sampling(i)]'];
                ddt=[zeros(size(t,1),1) zeros(size(t,1),1) ones(size(t,1),1)];
                dddt=[(1/frequency:1/frequency:experiment.phase_duration_sampling(i))]';
                
                
                xc_discret=[xc_discret;t*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt.^3/6*MPC_outputs_storage.xc_control(i)];
                xdc_discret=[xdc_discret;dt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt.^2/2*MPC_outputs_storage.xc_control(i)];
                xddc_discret=[xddc_discret;ddt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt*MPC_outputs_storage.xc_control(i)];

                yc_discret=[yc_discret;t*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt.^3/6*MPC_outputs_storage.yc_control(i)];
                ydc_discret=[ydc_discret;dt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt.^2/2*MPC_outputs_storage.yc_control(i)];
                yddc_discret=[yddc_discret;ddt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt*MPC_outputs_storage.yc_control(i)];

                zc_discret=[zc_discret;t*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt.^3/6*MPC_outputs_storage.zc_control(i)];
                zdc_discret=[zdc_discret;dt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt.^2/2*MPC_outputs_storage.zc_control(i)];
                zddc_discret=[zddc_discret;ddt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt*MPC_outputs_storage.zc_control(i)];
            end
        case 'comPolyExpo'
            xc_discret=[MPC_outputs_storage.xc(1)];
            xdc_discret=[MPC_outputs_storage.xdc(1)];
            xddc_discret=[MPC_outputs_storage.xddc(1)];

            yc_discret=[MPC_outputs_storage.yc(1)];
            ydc_discret=[MPC_outputs_storage.ydc(1)];
            yddc_discret=[MPC_outputs_storage.yddc(1)];

            zc_discret=[MPC_outputs_storage.zc(1)];
            zdc_discret=[MPC_outputs_storage.zdc(1)];
            zddc_discret=[MPC_outputs_storage.zddc(1)];
            for i=1:size(MPC_outputs_storage.xc_control,1)
                et=exp(-experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]');
                
                t=[1/2*(et.^2-4*et+3)*experiment.omega_temp^-1 1/2*(et-1).^2*experiment.omega_temp^-2];
                t=[ones(size(t,1),1) t];
                dt=[zeros(size(t,1),1) et.*(2-et) et.*(1-et)*experiment.omega_temp^-1];                
                ddt=[zeros(size(t,1),1) 2*et.*(et-1)*experiment.omega_temp et.*(2*et-1)];
                dddt=repmat((et.^-1-1),1,3).*[(et-1).^2 (1-et).*(2*et+1)*experiment.omega_temp (4*et.^2+et+1)*experiment.omega_temp^2];

            
                xc_discret=[xc_discret;t*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,1)*MPC_outputs_storage.xc_control(i)];
                xdc_discret=[xdc_discret;dt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,2)*MPC_outputs_storage.xc_control(i)];
                xddc_discret=[xddc_discret;ddt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,3)*MPC_outputs_storage.xc_control(i)];

                yc_discret=[yc_discret;t*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,1)*MPC_outputs_storage.yc_control(i)];
                ydc_discret=[ydc_discret;dt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,2)*MPC_outputs_storage.yc_control(i)];
                yddc_discret=[yddc_discret;ddt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,3)*MPC_outputs_storage.yc_control(i)];

                zc_discret=[zc_discret;t*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,1)*MPC_outputs_storage.zc_control(i)];
                zdc_discret=[zdc_discret;dt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,2)*MPC_outputs_storage.zc_control(i)];
                zddc_discret=[zddc_discret;ddt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,3)*MPC_outputs_storage.zc_control(i)];
            end
        case 'comExponential'
            xc_discret=[MPC_outputs_storage.xc(1)];
            xdc_discret=[MPC_outputs_storage.xdc(1)];
            xddc_discret=[MPC_outputs_storage.xddc(1)];

            yc_discret=[MPC_outputs_storage.yc(1)];
            ydc_discret=[MPC_outputs_storage.ydc(1)];
            yddc_discret=[MPC_outputs_storage.yddc(1)];

            zc_discret=[MPC_outputs_storage.zc(1)];
            zdc_discret=[MPC_outputs_storage.zdc(1)];
            zddc_discret=[MPC_outputs_storage.zddc(1)];
            for i=1:size(MPC_outputs_storage.xc_control,1)
                t=[sinh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')/experiment.omega_temp (cosh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')-1)/experiment.omega_temp^2];
                t=[ones(size(t,1),1) t];            
                dt=[zeros(size(t,1),1) cosh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]') sinh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')/experiment.omega_temp];
                ddt=[zeros(size(t,1),1) experiment.omega_temp*sinh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]') cosh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')];
                dddt=[-sinh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')/experiment.omega_temp+[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]' -cosh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')+1 -experiment.omega_temp*sinh(experiment.omega_temp*[1/frequency:1/frequency:experiment.phase_duration_sampling(i)]')];

            
                xc_discret=[xc_discret;t*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,1)*MPC_outputs_storage.xc_control(i)];
                xdc_discret=[xdc_discret;dt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,2)*MPC_outputs_storage.xc_control(i)];
                xddc_discret=[xddc_discret;ddt*[MPC_outputs_storage.xc(i);MPC_outputs_storage.xdc(i);MPC_outputs_storage.xddc(i)]+dddt(:,3)*MPC_outputs_storage.xc_control(i)];

                yc_discret=[yc_discret;t*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,1)*MPC_outputs_storage.yc_control(i)];
                ydc_discret=[ydc_discret;dt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,2)*MPC_outputs_storage.yc_control(i)];
                yddc_discret=[yddc_discret;ddt*[MPC_outputs_storage.yc(i);MPC_outputs_storage.ydc(i);MPC_outputs_storage.yddc(i)]+dddt(:,3)*MPC_outputs_storage.yc_control(i)];

                zc_discret=[zc_discret;t*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,1)*MPC_outputs_storage.zc_control(i)];
                zdc_discret=[zdc_discret;dt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,2)*MPC_outputs_storage.zc_control(i)];
                zddc_discret=[zddc_discret;ddt*[MPC_outputs_storage.zc(i);MPC_outputs_storage.zdc(i);MPC_outputs_storage.zddc(i)]+dddt(:,3)*MPC_outputs_storage.zc_control(i)];
            end
        otherwise
            error('Bad COM_form')
    end


% zzmp_ref_discret=zeros(round(max(phase_duration_cumul)*frequency),1);
zzmp_ref_discret=zeros(size(zc_discret,1)-1,1);
switch(walking_type)
    case 1
    case 2
        zzmp_ref_discret(round(phase_duration_cumul(5)*frequency)+1:round(phase_duration_cumul(7)*frequency))=0.195+0.18*0;
        zzmp_ref_discret(round(phase_duration_cumul(7)*frequency)+1:round(phase_duration_cumul(9)*frequency))=0.195+0.18*1;
        zzmp_ref_discret(round(phase_duration_cumul(9)*frequency)+1:round(phase_duration_cumul(11)*frequency))=0.195+0.18*2;
        zzmp_ref_discret(round(phase_duration_cumul(11)*frequency)+1:round(phase_duration_cumul(13)*frequency))=0.195+0.18*3;
        zzmp_ref_discret(round(phase_duration_cumul(13)*frequency)+1:end)=0.195+0.18*3+0.145;
    case 3
    case 4
    case 5
        zzmp_ref_discret(round(phase_duration_cumul(5)*frequency)+1:round(phase_duration_cumul(7)*frequency))=0.195+0.18*0;
        zzmp_ref_discret(round(phase_duration_cumul(7)*frequency)+1:round(phase_duration_cumul(9)*frequency))=0.195+0.18*1;
        zzmp_ref_discret(round(phase_duration_cumul(9)*frequency)+1:round(phase_duration_cumul(11)*frequency))=0.195+0.18*2;
        zzmp_ref_discret(round(phase_duration_cumul(11)*frequency)+1:round(phase_duration_cumul(13)*frequency))=0.195+0.18*3;
        zzmp_ref_discret(round(phase_duration_cumul(13)*frequency)+1:end)=0.195+0.18*3+0.145;
end

% zz=[zzmp_ref(1);zzmp_ref(1:end-1)];
% zz(length(zc)+1:end)=[];
zz_discret=[zzmp_ref_discret(1);zzmp_ref_discret(1:end)];

xz_discret=1*xc_discret+0*xdc_discret-(zc_discret-zz_discret)./(zddc_discret+experiment.g).*xddc_discret;
yz_discret=1*yc_discret+0*ydc_discret-(zc_discret-zz_discret)./(zddc_discret+experiment.g).*yddc_discret;
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