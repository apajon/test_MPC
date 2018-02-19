%% COM vel ref
xvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);
yvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);

xvcom_ref=xvcom_ref+0.2; %m.s^-1

xvcom_ref=[xvcom_ref;xvcom_ref(end)*ones(N,1)];
yvcom_ref=[yvcom_ref;yvcom_ref(end)*ones(N,1)];

%% COM height ref
hcom_ref=zeros(round(max(phase_duration_cumul)/T+N),1);

hcom_ref=hcom_ref+h_com;

%% ZMP height ref
zzmp_ref=zeros(round(max(phase_duration_cumul)/T+N),1);

%% zeta boundaries ref
zeta_up_ref=(hcom_ref+h_com_max)/g;

zeta_down_ref=(hcom_ref+h_com_min)/g;
