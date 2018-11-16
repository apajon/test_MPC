%% Initialize from last fixed foot step
%% Foot step ref
    Px_step_reduce=Px_step_ref(preview_windows,:);
    Px_step_reduce=Px_step_reduce(:,any(Px_step_reduce));
    
    phase_type_sampling_reduce=phase_type_sampling(preview_windows,:);
%     phase_type_sampling_reduce(any(phase_type_sampling_reduce=="start",2))='b';
%     phase_type_sampling_reduce(any(phase_type_sampling_reduce=="stop",2))='b';
    phase_type_reduce=phase_type_sampling_reduce(([0;phase_type_sampling_reduce(1:end-1)]==phase_type_sampling_reduce)==0);
    if length(phase_type_reduce)==1
        Pu_step=[];
        Px_step=Px_step_reduce;
    elseif length(phase_type_reduce)==2
        if phase_type_reduce(1)=='b' || phase_type_reduce(1)=="start" %|| phase_type_reduce(1)=="stop" %"stop cannot be followed by any phases type
            Pu_step=[];
            Px_step=Px_step_reduce(:,1:2);
        else
            Pu_step=Px_step_reduce(:,2);
            Px_step=Px_step_reduce(:,1);
        end
    else
        if phase_type_reduce(1)=='b' || phase_type_reduce(1)=="start" %|| phase_type_reduce(1)=='stop' %"stop cannot be followed by any phases type
            Pu_step=Px_step_reduce(:,3:end);
            Px_step=Px_step_reduce(:,1:2);
        else
            Pu_step=Px_step_reduce(:,2:end);
            Px_step=Px_step_reduce(:,1);
        end
    end
        
    H_step=Pu_step.'*Pu_step;
    
    if size(Px_step,2)==1
        xf_step=Px_step*xstep(end);
        yf_step=Px_step*ystep(end);
    else
        xf_step=Px_step*xstep(end-1:end,1);
        yf_step=Px_step*ystep(end-1:end,1);
    end
    
%% Foot step height ref
    zstep_ref_reduce=zzmp_ref(preview_windows,:);