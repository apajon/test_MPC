function [preview_window]=extract_preview_window(j,N,phase_duration_sampling,frequency,QP_result_all,zzmp_ref_discret,...
                                                                        Px_c,Pu_c,...
                                                                        Px_dc,Pu_dc,...
                                                                        Px_ddc,Pu_ddc,...
                                                                        xc,yc,zc,...
                                                                        xdc,ydc,zdc,...
                                                                        xddc,yddc,zddc,...
                                                                        COM_form,omega_temp,w_final)
%function to extract preview window and horizon
preview_window=[];
horizon_duration=1.5;
g=9.81;

xc_=Px_c*[xc(j);xdc(j);xddc(j)]+Pu_c*QP_result_all{j}(1:N);
yc_=Px_c*[yc(j);ydc(j);yddc(j)]+Pu_c*QP_result_all{j}((end-N)/2+1:(end-N)/2+N);
zc_=Px_c*[zc(j);zdc(j);zddc(j)]+Pu_c*QP_result_all{j}((end-N)+1:end);

xdc_=Px_dc*[xc(j);xdc(j);xddc(j)]+Pu_dc*QP_result_all{j}(1:N);
ydc_=Px_dc*[yc(j);ydc(j);yddc(j)]+Pu_dc*QP_result_all{j}((end-N)/2+1:(end-N)/2+N);
zdc_=Px_dc*[zc(j);zdc(j);zddc(j)]+Pu_dc*QP_result_all{j}((end-N)+1:end);

xddc_=Px_ddc*[xc(j);xdc(j);xddc(j)]+Pu_ddc*QP_result_all{j}(1:N);
yddc_=Px_ddc*[yc(j);ydc(j);yddc(j)]+Pu_ddc*QP_result_all{j}((end-N)/2+1:(end-N)/2+N);
zddc_=Px_ddc*[zc(j);zdc(j);zddc(j)]+Pu_ddc*QP_result_all{j}((end-N)+1:end);

preview_window.xc_discret_=[xc_(1)];
xdc_discret_=[xdc_(1)];
xddc_discret_=[xddc_(1)];

preview_window.yc_discret_=[yc_(1)];
ydc_discret_=[ydc_(1)];
yddc_discret_=[yddc_(1)];

preview_window.zc_discret_=[zc_(1)];
zdc_discret_=[zdc_(1)];
zddc_discret_=[zddc_(1)];

switch(COM_form)
    case 'com jerk'    
        for i=2:N
            t=[[1/frequency:1/frequency:phase_duration_sampling(i)]' [(1/frequency:1/frequency:phase_duration_sampling(i)).^2./2]'];
            t=[ones(size(t,1),1) t];
            dt=[zeros(size(t,1),1) ones(size(t,1),1) [1/frequency:1/frequency:phase_duration_sampling(i)]'];
            ddt=[zeros(size(t,1),1) zeros(size(t,1),1) ones(size(t,1),1)];
            dddt=[(1/frequency:1/frequency:phase_duration_sampling(i))]';


            preview_window.xc_discret_=[preview_window.xc_discret_;t*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt.^3/6*QP_result_all{j}(i)];
            xdc_discret_=[xdc_discret_;dt*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt.^2/2*QP_result_all{j}(i)];
            xddc_discret_=[xddc_discret_;ddt*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt*QP_result_all{j}(i)];

            preview_window.yc_discret_=[preview_window.yc_discret_;t*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt.^3/6*QP_result_all{j}((end-N)/2+i)];
            ydc_discret_=[ydc_discret_;dt*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt.^2/2*QP_result_all{j}((end-N)/2+i)];
            yddc_discret_=[yddc_discret_;ddt*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt*QP_result_all{j}((end-N)/2+i)];

            preview_window.zc_discret_=[preview_window.zc_discret_;t*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt.^3/6*QP_result_all{j}(end-N+i)];
            zdc_discret_=[zdc_discret_;dt*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt.^2/2*QP_result_all{j}(end-N+i)];
            zddc_discret_=[zddc_discret_;ddt*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt*QP_result_all{j}(end-N+i)];
        end
    case 'zmp vel'
            for i=2:N
                t=[sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp (cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')-1)/omega_temp^2];
                t=[ones(size(t,1),1) t];            
                dt=[zeros(size(t,1),1) cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]') sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp];
                ddt=[zeros(size(t,1),1) omega_temp*sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]') cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')];
                dddt=[-sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')/omega_temp+[1/frequency:1/frequency:phase_duration_sampling(i)]' -cosh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')+1 -omega_temp*sinh(omega_temp*[1/frequency:1/frequency:phase_duration_sampling(i)]')];


                preview_window.xc_discret_=[preview_window.xc_discret_;t*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt(:,1)*QP_result_all{j}(i)];
                xdc_discret_=[xdc_discret_;dt*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt(:,2)*QP_result_all{j}(i)];
                xddc_discret_=[xddc_discret_;ddt*[xc_(i-1);xdc_(i-1);xddc_(i-1)]+dddt(:,3)*QP_result_all{j}(i)];

                preview_window.yc_discret_=[preview_window.yc_discret_;t*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt(:,1)*QP_result_all{j}((end-N)/2+i)];
                ydc_discret_=[ydc_discret_;dt*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt(:,2)*QP_result_all{j}((end-N)/2+i)];
                yddc_discret_=[yddc_discret_;ddt*[yc_(i-1);ydc_(i-1);yddc_(i-1)]+dddt(:,3)*QP_result_all{j}((end-N)/2+i)];

                preview_window.zc_discret_=[preview_window.zc_discret_;t*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt(:,1)*QP_result_all{j}(end-N+i)];
                zdc_discret_=[zdc_discret_;dt*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt(:,2)*QP_result_all{j}(end-N+i)];
                zddc_discret_=[zddc_discret_;ddt*[zc_(i-1);zdc_(i-1);zddc_(i-1)]+dddt(:,3)*QP_result_all{j}(end-N+i)];
            end
end

preview_window.zz_discret_=[zzmp_ref_discret(1:301)];
% 
xz_=1*xc_+0*xdc_-(zc_)./(zddc_+g).*xddc_;
yz_=1*yc_+0*ydc_-(zc_)./(zddc_+g).*yddc_;

preview_window.xz_discret_=1*preview_window.xc_discret_+0*xdc_discret_-(preview_window.zc_discret_-preview_window.zz_discret_)./(zddc_discret_+g).*xddc_discret_;
preview_window.yz_discret_=1*preview_window.yc_discret_+0*ydc_discret_-(preview_window.zc_discret_-preview_window.zz_discret_)./(zddc_discret_+g).*yddc_discret_;


% [QP_result_all{j}(N+1) QP_result_all{j}((end-N)/2+N+1);QP_result_all{j}(N+2) QP_result_all{j}((end-N)/2+N+2)];
% XY=drawing_rectangle_rotate([QP_result_all{j}(N+1) QP_result_all{j}((end-N)/2+N+1);QP_result_all{j}(N+2) QP_result_all{j}((end-N)/2+N+2)],psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
% [QP_result_all{j}(N+1) QP_result_all{j}((end-N)/2+N+1);QP_result_all{j}(N+2) QP_result_all{j}((end-N)/2+N+2)];
% for k=1:2
%     plot(XY(k,1:5),XY(k,6:10),'-r','LineWidth',2)
% end
% 
% [QP_result_all{j}(N+1) QP_result_all{j}((end-N)/2+N+1);QP_result_all{j}(N+2) QP_result_all{j}((end-N)/2+N+2)];
% XY=drawing_rectangle_rotate([QP_result_all{j}(N+1) QP_result_all{j}((end-N)/2+N+1);QP_result_all{j}(N+2) QP_result_all{j}((end-N)/2+N+2)],psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
% for k=1:2
%     plot(XY(k,1:5),XY(k,6:10),':r','LineWidth',2)
% end

t_horizon=[0:1/frequency:horizon_duration];
exp_horizon=exp(-w_final*t_horizon);

preview_window.xalpha_horizon=preview_window.xc_discret_(end)+xdc_discret_(end)/w_final;
xgamma_horizon=xddc_discret_(end)/w_final^2;
preview_window.yalpha_horizon=preview_window.yc_discret_(end)+ydc_discret_(end)/w_final;
ygamma_horizon=yddc_discret_(end)/w_final^2;
preview_window.zalpha_horizon=preview_window.zc_discret_(end)+zdc_discret_(end)/w_final;
zgamma_horizon=zddc_discret_(end)/w_final^2;

preview_window.xc_discret_horizon=[preview_window.xalpha_horizon+xgamma_horizon*exp_horizon];
xdc_discret_horizon=[-w_final*xgamma_horizon*exp_horizon];
xddc_discret_horizon=[-w_final^2*xgamma_horizon*exp_horizon];

preview_window.yc_discret_horizon=[preview_window.yalpha_horizon+ygamma_horizon*exp_horizon];
ydc_discret_horizon=[-w_final*ygamma_horizon*exp_horizon];
yddc_discret_horizon=[-w_final^2*ygamma_horizon*exp_horizon];

preview_window.zc_discret_horizon=[preview_window.zalpha_horizon+zgamma_horizon*exp_horizon];
zdc_discret_horizon=[-w_final*zgamma_horizon*exp_horizon];
zddc_discret_horizon=[-w_final^2*zgamma_horizon*exp_horizon];

preview_window.zz_discret_horizon=[zzmp_ref_discret(1:length(preview_window.zc_discret_horizon))];
% 
preview_window.xz_discret_horizon=1*preview_window.xc_discret_horizon+0*xdc_discret_horizon-(preview_window.zc_discret_horizon-preview_window.zz_discret_horizon)./(zddc_discret_horizon+g).*xddc_discret_horizon;
preview_window.yz_discret_horizon=1*preview_window.yc_discret_horizon+0*ydc_discret_horizon-(preview_window.zc_discret_horizon-preview_window.zz_discret_horizon)./(zddc_discret_horizon+g).*yddc_discret_horizon;
    