%% COM vel ref
xvcom_ref=zeros(phase_duration_iteration_cumul(end),1);
yvcom_ref=zeros(phase_duration_iteration_cumul(end),1);

% vcom_change=round(size(xvcom_ref,1)*1/2)+4;
vcom_change=round(size(xvcom_ref,1)*1/3)+16;

switch(walking_type)
    case 1
        vcom_1=0.2; %m.s^-1
        vcom_2=0.5; %m.s^-1
    case 2
        vcom_1=0.375; %m.s^-1
        vcom_2=0.375; %m.s^-1
    case 3
        vcom_1=0.6; %m.s^-1
        vcom_2=1.2; %m.s^-1
    case 4
        vcom_1=0; %m.s^-1
        vcom_2=0; %m.s^-1
    case 5
        vcom_1=0; %m.s^-1
        vcom_2=0; %m.s^-1
end
xvcom_ref(N_start+1:vcom_change)=xvcom_ref(N_start+1:vcom_change)+vcom_1; %m.s^-1
xvcom_ref(vcom_change+1:end-N_stop)=xvcom_ref(vcom_change+1:end-N_stop)+vcom_2; %m.s^-1
% xvcom_ref(N_start+1:vcom_change)=xvcom_ref(N_start+1:vcom_change)+vcom_1; %m.s^-1
% xvcom_ref(vcom_change+1:vcom_change+8)=xvcom_ref(vcom_change+1:vcom_change+8)+(vcom_1+vcom_2)/2; %m.s^-1
% xvcom_ref(vcom_change+9:end-N_stop)=xvcom_ref(vcom_change+9:end-N_stop)+vcom_2; %m.s^-1


xvcom_ref=[xvcom_ref;zeros(size(phase_type_sampling,1)-size(xvcom_ref,1),1)];
yvcom_ref=[yvcom_ref;zeros(size(phase_type_sampling,1)-size(yvcom_ref,1),1)];

%% ZMP height ref
% zzmp_ref=zeros(round(max(phase_duration_cumul)/T+N),1);
% zzmp_ref(round(phase_duration_cumul(3)/T)+1:round(phase_duration_cumul(5)/T))=0.5;
% zzmp_ref(round(phase_duration_cumul(5)/T)+1:round(phase_duration_cumul(7)/T))=0.1;
% zzmp_ref(round(phase_duration_cumul(7)/T)+1:round(phase_duration_cumul(9)/T))=0.15;
% zzmp_ref(round(phase_duration_cumul(9)/T)+1:end)=0.2;

zzmp_ref=zeros(size(xvcom_ref,1),1);
switch(walking_type)
    case 1
    case 2
        zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=0.195+0.18*0;
        zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=0.195+0.18*1;
        zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=0.195+0.18*2;
        zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=0.195+0.18*3;
        zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=0.195+0.18*3+0.145;
        
%         zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=-0.195-0.18*0;
%         zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=-0.195-0.18*1;
%         zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=-0.195-0.18*2;
%         zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=-0.195-0.18*3;
%         zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=-0.195-0.18*3-0.145;



    case 3
    case 4
    case 5
        zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=0.195+0.18*0;
        zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=0.195+0.18*1;
        zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=0.195+0.18*2;
        zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=0.195+0.18*3;
        zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=0.195+0.18*3+0.145;
        
%         zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=-0.195-0.18*0;
%         zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=-0.195-0.18*1;
%         zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=-0.195-0.18*2;
%         zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=-0.195-0.18*3;
%         zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=-0.195-0.18*3-0.145;
end


%% COM height ref
hcom_ref=zeros(size(xvcom_ref,1),1);

hcom_ref=hcom_ref+h_com+zzmp_ref;

hcom_ref_max=hcom_ref;
hcom_ref_max(11:length(xvcom_ref)-40)=hcom_ref_max(11:length(xvcom_ref)-40)+h_com_max+0.2;

%% zeta boundaries ref
zeta_up_ref=(hcom_ref+h_com_max-zzmp_ref+0.32)/g;

zeta_down_ref=(hcom_ref+h_com_min-zzmp_ref-0.02)/g;

% % zeta_up_ref(round(phase_duration(1)/T)+1:vcom_change)=(hcom_ref(round(phase_duration(1)/T)+1:vcom_change)+h_com_max-zzmp_ref(round(phase_duration(1)/T)+1:vcom_change)+0.02)/g; %m.s^-1
% % zeta_up_ref(vcom_change+1:end-phase_duration_stop/T)=(hcom_ref(vcom_change+1:end-phase_duration_stop/T)+h_com_max-zzmp_ref(vcom_change+1:end-phase_duration_stop/T)+0.32)/g;
% 
% 
% temp_=logical([]);
% for i =1:length(phase_type_sampling)-1
%     if phase_type_sampling(i)=='b'&& phase_type_sampling(i+1)~='b'
%         temp_=[temp_;true];
%     else
%         temp_=[temp_;false];
%     end
% end
% temp_=[temp_;false];
% zeta_up_ref(temp_)=...
%     (hcom_ref(temp_)+...
%     h_com_max-...
%     zzmp_ref(temp_)+0.02)/g;
% 
% temp_=[false;temp_(1:end-1)];
% 
% % temp_=logical([]);
% % for i =2:length(phase_type_sampling)
% %     if phase_type_sampling(i-1)=='b'&& phase_type_sampling(i)~='b'
% %         temp_=[temp_;true];
% %     else
% %         temp_=[temp_;false];
% %     end
% % end
% % temp_=[false;temp_];
% zeta_up_ref(temp_)=...
%     (hcom_ref(temp_)+...
%     h_com_max-...
%     zzmp_ref(temp_)+0.02)/g;
% 
% temp_=[false([3 1]);temp_(1:end-3)];
% 
% zeta_down_ref(temp_)=...
%     (hcom_ref(temp_)+...
%     h_com_min-...
%     zzmp_ref(temp_)+0.12)/g;
% 
% temp_=[false;temp_(1:end-1)];
% 
% zeta_down_ref(temp_)=...
%     (hcom_ref(temp_)+...
%     h_com_min-...
%     zzmp_ref(temp_)+0.22)/g;

%% foot step height ref
zstep_l_0=zzmp_ref(1);
zstep_r_0=zzmp_ref(1);

zstep_r_ref=zzmp_ref;

zstep_l_ref=zzmp_ref;