%% COM vel ref
xvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);
yvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);

xvcom_ref=xvcom_ref+0.2; %m.s^-1

xvcom_ref=[xvcom_ref;xvcom_ref(end)*ones(N,1)];
yvcom_ref=[yvcom_ref;yvcom_ref(end)*ones(N,1)];