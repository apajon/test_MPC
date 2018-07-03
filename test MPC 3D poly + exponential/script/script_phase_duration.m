%% Phase duration
% Phase type deifnition
%b = both feet; r = right foot; l = left foot
% phase_type=['b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b'];
phase_type=['b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b'];
% phase_type=['b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b'];
phase_type=['b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b'];

% Phase duration definition
phase_duration=zeros(length(phase_type),1);
phase_duration(any(phase_type=='r',2))=phase_duration_r;
phase_duration(any(phase_type=='l',2))=phase_duration_l;
phase_duration(any(phase_type=='b',2))=phase_duration_b;
phase_duration(1)=phase_duration_start;
if phase_type(end)=='b'
    phase_duration(end)=phase_duration_stop;
end

% Phase duration cumulative
phase_duration_cumul=zeros(length(phase_duration),1);
for i=1:length(phase_duration)
    phase_duration_cumul(i,1)=sum(phase_duration(1:i,1));
end

% Phase type sampling
phase_type_sampling=[];
for i=1:length(phase_duration)
    phase_type_sampling=[phase_type_sampling;repmat(phase_type(i),round(phase_duration(i)/T),1)];
end
phase_type_sampling=[phase_type_sampling;repmat(phase_type(end),N,1)];

% phase decoupling
phase_sampling_length=find(diff([0;phase_type_sampling])~=0);
phase_type_decouple=zeros(length(phase_type_sampling),length(phase_type));
for i=1:length(phase_sampling_length)-1
    phase_type_decouple(:,i)=[zeros(phase_sampling_length(i)-1,1);ones(phase_sampling_length(i+1)-phase_sampling_length(i),1);zeros(length(phase_type_sampling)-phase_sampling_length(i+1)+1,1)];
end
phase_type_decouple(:,end)=[zeros(phase_sampling_length(end)-1,1);ones(length(phase_type_sampling)-phase_sampling_length(end)+1,1)];

phase_type_decouple(:,1:2:end)=phase_type_decouple(:,1:2:end)./2;

Px_step_rl=phase_type_decouple(:,2:2:end);
Px_step_b=phase_type_decouple(:,1:2:end);

for i=2:size(Px_step_b,2)
    Px_step_rl(find(Px_step_b(:,i),1),i-1)=1;
    Px_step_b(find(Px_step_b(:,i),1),i)=0;
end

Px_step_ref=[Px_step_b zeros(size(Px_step_b,1),1)]+[zeros(size(Px_step_b,1),1) Px_step_b];
if phase_type(end)=='b'
    Px_step_ref(:,2:end-1)=Px_step_ref(:,2:end-1)+Px_step_rl;
else
    Px_step_ref(:,2:end)=Px_step_ref(:,2:end)+Px_step_rl;
end
