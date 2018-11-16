%% write txt 
e=0;
switch controller
    case 'vrep'
        e_=0;
    case 'rviz'
%             e_=0.093;
        e_=0.099;
    otherwise
        msg='Bad choice of controller \n';
        msg1='vrep \n';
        msg2='rviz \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end
%%
switch controller
    case 'vrep'
        trajectories=[dt_type_phase_(1:end-20) ...
            xz_discret yz_discret zz_discret ...-0.0317 ...
            xc_discret yc_discret zc_discret ...-0.0317 ...
            xdc_discret ydc_discret zdc_discret ...
            xddc_discret yddc_discret zddc_discret ...
            pankle_l(1:end-20,1) pankle_l(1:end-20,2) pankle_l(1:end-20,3)+e_-e ...
            vankle_l(1:end-20,1) vankle_l(1:end-20,2) vankle_l(1:end-20,3) ...
            aankle_l(1:end-20,1) aankle_l(1:end-20,2) aankle_l(1:end-20,3) ...
            pankle_r(1:end-20,1) pankle_r(1:end-20,2) pankle_r(1:end-20,3)+e_-e ...
            vankle_r(1:end-20,1) vankle_r(1:end-20,2) vankle_r(1:end-20,3) ...
            aankle_r(1:end-20,1) aankle_r(1:end-20,2) aankle_r(1:end-20,3) ...
            pankle_l(1:end-20,1)*0 pankle_l(1:end-20,2)*0 pankle_l(1:end-20,3)*0 ... %rtheta_dt_r rphi_dt_r
            pankle_r(1:end-20,1)*0 pankle_r(1:end-20,2)*0 pankle_r(1:end-20,3)*0]; %rtheta_dt_r rphi_dt_r
    case 'rviz'
        trajectories=[dt_type_phase_(1:end-20)...
            xz_discret yz_discret zz_discret ...
            xc_discret yc_discret zc_discret ...
            xdc_discret ydc_discret zdc_discret ...
            xddc_discret yddc_discret zddc_discret ...
            pankle_l(1:end-20,1) pankle_l(1:end-20,2) pankle_l(1:end-20,3)+e_-e ...
            vankle_l(1:end-20,1) vankle_l(1:end-20,2) vankle_l(1:end-20,3) ...
            aankle_l(1:end-20,1) aankle_l(1:end-20,2) aankle_l(1:end-20,3) ...
            pankle_r(1:end-20,1) pankle_r(1:end-20,2) pankle_r(1:end-20,3)+e_-e ...
            vankle_r(1:end-20,1) vankle_r(1:end-20,2) vankle_r(1:end-20,3) ...
            aankle_r(1:end-20,1) aankle_r(1:end-20,2) aankle_r(1:end-20,3) ...
            pankle_l(1:end-20,1)*0 pankle_l(1:end-20,2)*0 pankle_l(1:end-20,3)*0 ... %rtheta_dt_r rphi_dt_r
            pankle_r(1:end-20,1)*0 pankle_r(1:end-20,2)*0 pankle_r(1:end-20,3)*0]; %rtheta_dt_r rphi_dt_r
    otherwise
        msg='Bad choice of controller \n';
        msg1='vrep \n';
        msg2='rviz \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end



%% %save data in txt
% zmpcom=fopen('zmp_com_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');
filename=['zmp_com_type_' num2str(walking_type) '_controller_' controller '_vcom1_' num2str(vcom_1) '_vcom2_' num2str(vcom_2)];
filename=strrep(filename,'.','_');
filename=[filename '.txt'];
zmpcom=fopen(filename,'w');


for i=1:size(trajectories,1)
    fprintf(zmpcom,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',trajectories(i,:));
end
fclose(zmpcom);

%%

switch controller
    case 'vrep'
        pstep_=[pstep(:,1) pstep(:,2) zstep];
    case 'rviz'
        psi_=pstep(:,1)*0;
        pstep_=[pstep(:,1) pstep(:,2) psi_];
    otherwise
        msg='Bad choice of controller \n';
        msg1='vrep \n';
        msg2='rviz \n';
        errormsg=[msg msg1 msg2];
        error(errormsg,[])
end

% pstepf=fopen('pstep_9_100_step4_oscil16cm_tss1000_tds500_tpi2500.txt','w');
filename=['pstep_type_' num2str(walking_type) '_controller_' controller '_vcom1_' num2str(vcom_1) '_vcom2_' num2str(vcom_2)];
filename=strrep(filename,'.','_');
filename=[filename '.txt'];
pstepf=fopen(filename,'w');


for i=1:size(pstep_,1)
    fprintf(pstepf,'%f %f %f\n',pstep_(i,:));
end
fclose(pstepf);

fclose('all');
%%%%%%%%%%%