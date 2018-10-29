%% Phase duration
%% Phase type definition
%b = both feet; r = right foot; l = left foot
% phase_type=["start";'r';'b';'l';'b';'r';'b';'l';"stop"];
% phase_type=["start";'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';"stop"];
phase_type=["start";'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';"stop"];
% phase_type=["start";'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';"stop"];
% phase_type=["start";'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';"stop"];

%% fixed step position after initial robot step state position
step_number_pankle_fixed=[];

switch(walking_type)
    case 1
    case 2
    case 3
    case 4
%     step_number_pankle_fixed=[1 0.179769020726667 0.08165;
%         2 0.352958329101274 -0.08165;
%         3 0.510495932924150 0.08165;
%         4 0.669818528151903 -0.08165;
%         5 0.820250691525008 0.08165;
%         6 0.954993759747009 -0.08165;
%         7 1.43427808991043 0.08165;
%         8 1.89901923954723 -0.08165;
%         9 2.36638271416171 0.08165;
%         10 2.83312526584211 -0.08165;
%         11 3.33584997976809 0.08165;
%         12 3.76142734060293 -0.08165];
    
%     step_number_pankle_fixed=[1  0.2263    0.08165;
%        2  0.3566   -0.08165;
%        3  0.5410    0.08165;
%        4  1.0163   -0.08165;
%        5  1.4800    0.08165;
%        6  1.9454   -0.08165;
%        7  2.4105    0.08165;
%        8  2.8756   -0.08165;
%        9  3.3408    0.08165;
%        10 3.8056   -0.08165;
%        11 4.3042    0.08165;
%        12 4.7332   -0.08165];

    step_number_pankle_fixed=[1 0.2186    0.0948
    2 0.3742   -0.0684
    3 0.5340    0.0948
    4 0.6640   -0.0684
    5 0.8498    0.0948
    6 1.3324   -0.0684
    7 1.7962    0.0948
    8 2.2638   -0.0684
    9 2.7306    0.0948
    10 3.1973   -0.0684
    11 3.7004    0.0948
    12 4.1259   -0.0684];
    case 5
    step_number_pankle_fixed=[1 0.545705354503507 0.08165;
        2 0.923523675671500 -0.08165;
        3 1.331722881724974 0.08165;
        4 1.731070868822926 -0.08165;
        5 2.112447651732674 0.08165;
        6 2.571371588285472 -0.08165;
        7 3.040970201054318 0.08165;
        8 3.507119849382747 -0.08165;
        9 3.974022092529370 0.08165;
        10 4.440756030388395 -0.08165;
        11 4.943227783727431 0.08165;
        12 5.368832127933287 -0.08165];
%     
%     step_number_pankle_fixed=[1 0.607949782746166 0.08165;
%         2 0.972852618981015 -0.08165;
%         3 1.348250593961038 0.08165;
%         4 1.715487326299075 -0.08165;
%         5 2.080872866572241 0.08165;
%         6 2.506027382745452 -0.08165;
%         7 2.985104090834402 0.08165;
%         8 3.454177111630078 -0.08165;
%         9 3.926851362749752 0.08165;
%         10 4.398311269525593 -0.08165;
%         11 4.903572428340929 0.08165;
%         12 5.225578480707858 -0.08165];

    step_number_pankle_fixed=[1 0.5 0.08165;
        2 0.5+0.28 NaN;
        3 0.5+0.28*2 NaN;
        4 0.5+0.28*3 NaN;
        5 0.5+0.28*4 NaN;
        6 0.5+0.28*5 NaN;
        7 0.5+0.28*5+0.4 NaN;
        8 0.5+0.28*5+0.4*2 NaN;
        9 0.5+0.28*5+0.4*3 NaN;
        10 0.5+0.28*5+0.4*4 -0.08165;
        11 0.5+0.28*5+0.4*5 0.08165;
        12 0.5+0.28*5+0.4*6 -0.08165];
    
    step_number_pankle_fixed=[1 0.335058585475008   0.084110254671146
       2 0.625058010222233  -0.079089745329704
       3 0.923682675664285   0.084110254670411
       4 1.219052125317584  -0.079089745330432
       5 1.515441484526088   0.084110254678538
       6 1.809673827281263  -0.079089745321813
       7 2.108828040870965   0.084110254682851
       8 2.405016021874720  -0.079089745317367
       9 2.702365606329494   0.084110254683305
       10 2.999236624296849  -0.079089745317735
       11 3.325086673919420   0.084110254682445
       12 3.554027657071293  -0.079089745362955];

    
    step_number_pankle_fixed=[1 0.335058585475008   NaN
       2 0.335058585475008+0.24  NaN
       3 0.335058585475008+0.24*2  NaN
       4 0.335058585475008+0.24*3  NaN
       5 0.335058585475008+0.24*4  NaN
       6 1.809673827281263+0.24*5  NaN
       7 2.108828040870965  NaN
       8 2.405016021874720  NaN
       9 2.702365606329494  NaN
       10 2.999236624296849  NaN
       11 3.325086673919420  NaN
       12 3.554027657071293  NaN];
   step_number_pankle_fixed(:,2)=0.335058585475008+0.24*[0:11]';


end




%% Phase duration definition
phase_duration=zeros(length(phase_type),1);
phase_duration(any(phase_type=='r',2))=phase_duration_r;
phase_duration(any(phase_type=='l',2))=phase_duration_l;
phase_duration(any(phase_type=='b',2))=phase_duration_b;
phase_duration(any(phase_type=='start',2))=phase_duration_start;
phase_duration(any(phase_type=='stop',2))=phase_duration_stop;

phase_duration_iteration=zeros(length(phase_type),1);
phase_duration_iteration(any(phase_type=='r',2))=N_r;
phase_duration_iteration(any(phase_type=='l',2))=N_l;
phase_duration_iteration(any(phase_type=='b',2))=N_b;
phase_duration_iteration(any(phase_type=='start',2))=N_start;
phase_duration_iteration(any(phase_type=='stop',2))=N_stop;


%% Phase duration cumulative
phase_duration_cumul=cumsum(phase_duration);
phase_duration_iteration_cumul=cumsum(phase_duration_iteration);

%% Phase type sampling
phase_type_sampling=[];
for i=1:length(phase_duration)
    switch(phase_type(i))
        case 'start'
            phase_type_sampling_temp=repmat(phase_type(i),N_start,1);
        case 'stop'
            phase_type_sampling_temp=repmat(phase_type(i),N_stop,1);
        case 'r'
            phase_type_sampling_temp=repmat(phase_type(i),N_r,1);
        case 'l'
            phase_type_sampling_temp=repmat(phase_type(i),N_l,1);
        case 'b'
            phase_type_sampling_temp=repmat(phase_type(i),N_b,1);
        otherwise
            error(['Unknown phase type of phase_type(i) with i = ' num2str(i)])
    end
    phase_type_sampling=[phase_type_sampling;phase_type_sampling_temp];        
end

%add phase type sampling for the last preview windows
switch(phase_type(end))
    case 'start'
        phase_type_sampling_temp=repmat(phase_type(end),ceil(preview_windows_duration/T_start),1);
    case 'stop'
        phase_type_sampling_temp=repmat(phase_type(end),ceil(preview_windows_duration/T_stop),1);
    case 'r'
        phase_type_sampling_temp=repmat(phase_type(end),ceil(preview_windows_duration/T_r),1);
    case 'l'
        phase_type_sampling_temp=repmat(phase_type(end),ceil(preview_windows_duration/T_l),1);
    case 'b'
        phase_type_sampling_temp=repmat(phase_type(end),ceil(preview_windows_duration/T_b),1);
    otherwise
        error(['Unknown phase type of phase_type(i) with i = ' num2str(i)])
end
phase_type_sampling=[phase_type_sampling;phase_type_sampling_temp];

clear phase_type_sampling_temp

%% Phase sampling duration
phase_duration_sampling=zeros(length(phase_type_sampling),1);
phase_duration_sampling(any(phase_type_sampling=="start",2))=T_start;
phase_duration_sampling(any(phase_type_sampling=="stop",2))=T_stop;
phase_duration_sampling(any(phase_type_sampling=='r',2))=T_l;
phase_duration_sampling(any(phase_type_sampling=='l',2))=T_l;
phase_duration_sampling(any(phase_type_sampling=='b',2))=T_b;

if sum(any(phase_duration_sampling==0,2))~=0
    error(['Undefined phase type in phase_type_sampling'])
end

phase_duration_sampling_cumul=cumsum(phase_duration_sampling);

%% phase decoupling
phase_sampling_length=[1;cumsum(phase_duration_iteration(1:end-1))+1];

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
if phase_type(end)=='stop' ||  phase_type(end)=='b'
    Px_step_ref(:,2:end-1)=Px_step_ref(:,2:end-1)+Px_step_rl;
else
    Px_step_ref(:,2:end)=Px_step_ref(:,2:end)+Px_step_rl;
end
