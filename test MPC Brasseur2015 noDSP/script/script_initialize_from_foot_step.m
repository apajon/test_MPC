%% Initialize from last fixed foot step
%% Foot step ref
    Px_step_reduce=Px_step_ref(1+(i-1):N+(i-1),:);
    Px_step_reduce=Px_step_reduce(:,any(Px_step_reduce));
    
    phase_type_sampling_reduce=phase_type_sampling(i:N+(i-1));
    phase_type_reduce=phase_type_sampling_reduce(diff([0;phase_type_sampling_reduce])~=0);
    if length(phase_type_reduce)==1
        Pu_step=[];
        Px_step=Px_step_reduce;
    elseif length(phase_type_reduce)==2
        if phase_type_reduce(1)=='b'
            Pu_step=[];
            Px_step=Px_step_reduce(:,1:2);
        else
            Pu_step=Px_step_reduce(:,2);
            Px_step=Px_step_reduce(:,1);
        end
    else
        if phase_type_reduce(1)=='b'
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