%% Results ZMP
zz=[experiment.zfloor_ref(1);experiment.zfloor_ref(1:end-1)];
zz(length(MPC_outputs_storage.zc)+1:end)=[];
xz=1*MPC_outputs_storage.xc+0*MPC_outputs_storage.xdc-(MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g).*MPC_outputs_storage.xddc;
yz=1*MPC_outputs_storage.yc+0*MPC_outputs_storage.ydc-(MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g).*MPC_outputs_storage.yddc;

zz_up=zz;
xz_up=[xz(1);1*MPC_outputs_storage.xc(2:end)+0*MPC_outputs_storage.xdc(2:end)-experiment.zeta_up_ref(1:length(MPC_outputs_storage.zc)-1).*MPC_outputs_storage.xddc(2:end)];
yz_up=[yz(1);1*MPC_outputs_storage.yc(2:end)+0*MPC_outputs_storage.ydc(2:end)-experiment.zeta_up_ref(1:length(MPC_outputs_storage.zc)-1).*MPC_outputs_storage.yddc(2:end)];

zz_down=zz;
xz_down=1*MPC_outputs_storage.xc+0*MPC_outputs_storage.xdc-experiment.zeta_down_ref(1:length(MPC_outputs_storage.zc)).*MPC_outputs_storage.xddc;
yz_down=1*MPC_outputs_storage.yc+0*MPC_outputs_storage.ydc-experiment.zeta_down_ref(1:length(MPC_outputs_storage.zc)).*MPC_outputs_storage.yddc;

zcapture=zz;
xcapture=MPC_outputs_storage.xc+((MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g)).^(1/2).*MPC_outputs_storage.xdc;
ycapture=MPC_outputs_storage.yc+((MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g)).^(1/2).*MPC_outputs_storage.ydc;

xdcm=MPC_outputs_storage.xc+((MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g)).^(1/2).*MPC_outputs_storage.xdc;
ydcm=MPC_outputs_storage.yc+((MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g)).^(1/2).*MPC_outputs_storage.ydc;
zdcm=MPC_outputs_storage.zc+((MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g)).^(1/2).*MPC_outputs_storage.zdc;

zeta=(MPC_outputs_storage.zc-zz)./(MPC_outputs_storage.zddc+experiment.g);