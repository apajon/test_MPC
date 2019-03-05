function [A,b]=function_constraint_foot_stretching(MPC_inputs,Step_state_preview,xstep,ystep)
%% Constraint foot step stretching
    A_step_stretch=[];
    b_step_stretch=[];
    xb_step_stretch=[];yb_step_stretch=[];
    
    if size(Step_state_preview.Pu_step,2)>=1
        A_step_stretch=eye(size(Step_state_preview.Pu_step,2));
        A_step_stretch(2:end,1:end-1)=A_step_stretch(2:end,1:end-1)-eye(size(Step_state_preview.Pu_step,2)-1);
        A_step_stretch=[zeros(size(Step_state_preview.Pu_step,2),MPC_inputs.N) A_step_stretch];
        A_step_stretch=[A_step_stretch;-A_step_stretch];
        
        A_step_stretch=blkdiag(A_step_stretch*0,A_step_stretch);
        
        %%%
        phase_type_nodouble_reduce=MPC_inputs.phase_type_reduce(any(MPC_inputs.phase_type_reduce~='b'&MPC_inputs.phase_type_reduce~="start"&MPC_inputs.phase_type_reduce~="stop",2));
        
        if MPC_inputs.phase_type_reduce(end)=="stop" && Step_state_preview.Pu_step(end,end)==0.5
            if phase_type_nodouble_reduce(end)=='r'
                phase_type_nodouble_reduce(end+1)='l';
            else
                phase_type_nodouble_reduce(end+1)='r';
            end
        end
        
        phase_type_nodouble_reduce(1)=[];
        %%%
        xb_step_stretch=[ones(size(Step_state_preview.Pu_step,2),1)*MPC_inputs.xankmax;...
            ones(size(Step_state_preview.Pu_step,2),1)*(-MPC_inputs.xankmin)];
        xb_step_stretch([1 end/2+1])=xb_step_stretch([1 end/2+1])+[xstep(end);-xstep(end)];
 
        yb_step_stretch=[any(phase_type_nodouble_reduce=='l',2)*MPC_inputs.yankmax-any(phase_type_nodouble_reduce=='r',2)*MPC_inputs.yankmin;...
            any(phase_type_nodouble_reduce=='l',2)*(-MPC_inputs.yankmin)-any(phase_type_nodouble_reduce=='r',2)*(-MPC_inputs.yankmax)];
        yb_step_stretch([1 end/2+1])=yb_step_stretch([1 end/2+1])+[ystep(end);-ystep(end)];
    end
    
    b_step_stretch=[xb_step_stretch*0;yb_step_stretch];
    
    
    A=A_step_stretch;
    b=b_step_stretch;