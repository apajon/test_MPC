%% Results ZMP
zz=[experiment.zfloor_ref(1);experiment.zfloor_ref(1:end-1)];
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