classdef classdef_FootStep_state_preview<handle
    %%
    properties (SetAccess = protected)
        %linear matrix such as c=Px_c*c_init+Pu_c*X
        %where X is the associated vector of control parameters
        %step for foot step position
        
        Pu_step
        Px_step
        
        %vector of f_c=Px_c*c_init
        f_step
        
    end
    %%
    methods
        %%
        function obj=classdef_FootStep_state_preview(MPC_inputs,xstep,ystep)
            obj.update_Footstep(MPC_inputs);
            obj.update_initial_state(xstep,ystep);
        end
        %%
        function obj=update_Footstep(obj,MPC_inputs)
            if length(MPC_inputs.phase_type_reduce)==1
                obj.Pu_step=[];
                obj.Px_step=MPC_inputs.Px_step;
            elseif length(MPC_inputs.phase_type_reduce)==2
                if MPC_inputs.phase_type_reduce(1)=='b' || MPC_inputs.phase_type_reduce(1)=="start" %|| MPC_inputs.phase_type_reduce(1)=="stop" %"stop cannot be followed by any phases type
                    obj.Pu_step=[];
                    obj.Px_step=MPC_inputs.Px_step(:,1:2);
                else
                    obj.Pu_step=MPC_inputs.Px_step(:,2);
                    obj.Px_step=MPC_inputs.Px_step(:,1);
                end
            else
                if MPC_inputs.phase_type_reduce(1)=='b' || MPC_inputs.phase_type_reduce(1)=="start" %|| MPC_inputs.phase_type_reduce(1)=='stop' %"stop cannot be followed by any phases type
                    obj.Pu_step=MPC_inputs.Px_step(:,3:end);
                    obj.Px_step=MPC_inputs.Px_step(:,1:2);
                else
                    obj.Pu_step=MPC_inputs.Px_step(:,2:end);
                    obj.Px_step=MPC_inputs.Px_step(:,1);
                end
            end
        end
        %%
        function []=update_initial_state(obj,xstep,ystep)
            if size(obj.Px_step,2)==1
                obj.f_step(:,1)=obj.Px_step*xstep(end);
                obj.f_step(:,2)=obj.Px_step*ystep(end);
            else
                obj.f_step(:,1)=obj.Px_step*xstep(end-1:end,1);
                obj.f_step(:,2)=obj.Px_step*ystep(end-1:end,1);
            end

        end
    end
end