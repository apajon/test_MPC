%% Init storage QP result
% COM along x axis
% xc=zeros(N+1,1);
% xdc=zeros(N+1,1);
% xddc=zeros(N+1,1);
xdddc_storage=[];

xc(1)=xcom_0(1);
xdc(1)=xcom_0(2);
xddc(1)=xcom_0(3);

% COM along y axis
% yc=zeros(N+1,1);
% ydc=zeros(N+1,1);
% yddc=zeros(N+1,1);
ydddc_storage=[];

yc(1)=ycom_0(1);
ydc(1)=ycom_0(2);
yddc(1)=ycom_0(3);

% COM along z axis
% zc=zeros(N+1,1);
% zdc=zeros(N+1,1);
% zddc=zeros(N+1,1);
zdddc_storage=[];

zc(1)=zcom_0(1);
zdc(1)=zcom_0(2);
zddc(1)=zcom_0(3);

% foot step
% xstep=zeros(N+1,1);
% ystep=zeros(N+1,1);

if phase_type(2)=='r'
    xstep=[xstep_l_0;xstep_r_0];
    ystep=[ystep_l_0;ystep_r_0];
    zstep=[zstep_l_0;zstep_r_0];
elseif phase_type(2)=='l'
    xstep=[xstep_r_0;xstep_l_0];
    ystep=[ystep_r_0;ystep_l_0];
    zstep=[zstep_r_0;zstep_l_0];
end

% % Capture point
% xcapture=zeros(N+1,1);
% ycapture=zeros(N+1,1);
% 
% %zmp along x-axis
% xz=zeros(N+1,1);
% xdz=zeros(N+1,1);
% 
% xz(1)=xcom_0(1);
% 
% %zmp along y-axis
% yz=zeros(N+1,1);
% ydz=zeros(N+1,1);
% 
% yz(1)=ycom_0(1);
