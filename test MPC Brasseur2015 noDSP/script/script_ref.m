%% COM vel ref
xvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);
yvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);

vcom_change=round(size(xvcom_ref,1)/2);
xvcom_ref(1:vcom_change)=xvcom_ref(1:vcom_change)+0.2; %m.s^-1
xvcom_ref(vcom_change+1:end)=xvcom_ref(vcom_change+1:end)+0.6; %m.s^-1

xvcom_ref=[xvcom_ref;xvcom_ref(end)*ones(N,1)];
yvcom_ref=[yvcom_ref;yvcom_ref(end)*ones(N,1)];

%% ZMP height ref
zzmp_ref=zeros(round(max(phase_duration_cumul)/T+N),1);
zzmp_ref(round(phase_duration_cumul(3)/T)+1:round(phase_duration_cumul(5)/T))=0.05;
zzmp_ref(round(phase_duration_cumul(5)/T)+1:round(phase_duration_cumul(7)/T))=0.1;
zzmp_ref(round(phase_duration_cumul(7)/T)+1:round(phase_duration_cumul(9)/T))=0.15;
zzmp_ref(round(phase_duration_cumul(9)/T)+1:end)=0.2;

%% COM height ref
hcom_ref=zeros(round(max(phase_duration_cumul)/T+N),1);

hcom_ref=hcom_ref+h_com+zzmp_ref;

hcom_ref_max=hcom_ref;
hcom_ref_max(11:119)=hcom_ref_max(11:119)+h_com_max+0.1;

%% zeta boundaries ref
zeta_up_ref=(hcom_ref+h_com_max-zzmp_ref+0.02)/g;

zeta_down_ref=(hcom_ref+h_com_min-zzmp_ref-0.02)/g;

%% foot step height ref
zstep_l_0=zzmp_ref(1);
zstep_r_0=zzmp_ref(1);

zstep_r_ref=zzmp_ref;

zstep_l_ref=zzmp_ref;