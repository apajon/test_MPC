%% discretization trajectories
switch(COM_form)
        case 'com jerk'
            xc_discret=[xc(1)];
            xdc_discret=[xdc(1)];
            xddc_discret=[xddc(1)];

            yc_discret=[yc(1)];
            ydc_discret=[ydc(1)];
            yddc_discret=[yddc(1)];

            zc_discret=[zc(1)];
            zdc_discret=[zdc(1)];
            zddc_discret=[zddc(1)];
            for i=1:size(xdddc_storage,1)
                t=[[1/frequency:1/frequency:phase_duration_sampling(i)]' [(1/frequency:1/frequency:phase_duration_sampling(i)).^2./2]'];
                t=[ones(size(t,1),1) t];
                dt=[zeros(size(t,1),1) ones(size(t,1),1) [1/frequency:1/frequency:phase_duration_sampling(i)]'];
                ddt=[zeros(size(t,1),1) zeros(size(t,1),1) ones(size(t,1),1)];
                dddt=[(1/frequency:1/frequency:phase_duration_sampling(i))]';
                
                
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
        case 'poly expo'
            xc_discret=[xc(1)];
            xdc_discret=[xdc(1)];
            xddc_discret=[xddc(1)];

            yc_discret=[yc(1)];
            ydc_discret=[ydc(1)];
            yddc_discret=[yddc(1)];

            zc_discret=[zc(1)];
            zdc_discret=[zdc(1)];
            zddc_discret=[zddc(1)];
            for i=1:size(xdddc_storage,1)
                et=exp(-omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]');
                
                t=[1/2*(et.^2-4*et+3)*omega_temp^-1 1/2*(et-1).^2*omega_temp^-2];
                t=[ones(size(t,1),1) t];
                dt=[zeros(size(t,1),1) et.*(2-et) et.*(1-et)*omega_temp^-1];                
                ddt=[zeros(size(t,1),1) 2*et.*(et-1)*omega_temp et.*(2*et-1)];
                dddt=repmat((et.^-1-1),1,3).*[(et-1).^2 (1-et).*(2*et+1)*omega_temp (4*et.^2+et+1)*omega_temp^2];

            
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
        case 'zmp vel'
            xc_discret=[xc(1)];
            xdc_discret=[xdc(1)];
            xddc_discret=[xddc(1)];

            yc_discret=[yc(1)];
            ydc_discret=[ydc(1)];
            yddc_discret=[yddc(1)];

            zc_discret=[zc(1)];
            zdc_discret=[zdc(1)];
            zddc_discret=[zddc(1)];
            for i=1:size(xdddc_storage,1)
                t=[sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp (cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')-1)/omega_temp^2];
                t=[ones(size(t,1),1) t];            
                dt=[zeros(size(t,1),1) cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]') sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp];
                ddt=[zeros(size(t,1),1) omega_temp*sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]') cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')];
                dddt=[-sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp+[1/frequency:1/frequency:phase_duration_sampling(i)]' -cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')+1 -omega_temp*sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')];

            
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