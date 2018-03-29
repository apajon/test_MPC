%% COM vel ref
xvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);
yvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);

vcom_change=round(size(xvcom_ref,1)/2);
xvcom_ref(1:vcom_change)=xvcom_ref(1:vcom_change)+0.2; %m.s^-1
xvcom_ref(vcom_change+1:end)=xvcom_ref(vcom_change+1:end)+0.6; %m.s^-1

xvcom_ref=[xvcom_ref;xvcom_ref(end)*ones(N,1)];
yvcom_ref=[yvcom_ref;yvcom_ref(end)*ones(N,1)];

%% COM height ref
hcom_ref=zeros(round(max(phase_duration_cumul)/T+N),1);

hcom_ref=hcom_ref+h_com;

%% ZMP height ref
zzmp_ref=zeros(round(max(phase_duration_cumul)/T+N),1);
zzmp_ref(round(phase_duration_cumul(1:3)/T):round(phase_duration_cumul(1:5)/T))=0;

%% zeta boundaries ref
zeta_up_ref=(hcom_ref+h_com_max+0.02)/g;
% zeta_up_ref=hcom_ref/(-2+g);

zeta_down_ref=(hcom_ref+h_com_min-0.02)/g;
