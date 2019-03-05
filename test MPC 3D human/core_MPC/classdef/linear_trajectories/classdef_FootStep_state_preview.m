classdef classdef_FootStep_state_preview<handle
    %Class in the MPC core
    %
    %This class create the iterative matrix of support foot position at the end of
    %each iteration of the preview window
    %
    %The trajectories are linear with the control parameters
    %such as step = Px_step*step_init + Pu_c*X
    %where X is the associated vector of control parameters (foot step)
    %and step_init is a vector of the known foot step positions
    %at the beginning of the preview window
    %
    %NOTE the simplification
    %step for foot step position
    
    %%
    properties (SetAccess = protected)
        Pu_step %Px matrix of the foot step position
        Px_step %Pu matrix of the foot step position
        
        f_step %matrix of f_step=Px_step*step_init where step_init are the known foot step position at the beginning of the preview window along each direction within each collumn
        
    end
    %%
    methods       
        %%
        function obj=classdef_FootStep_state_preview(MPC_inputs,xstep,ystep)
            %Creator of the classdef_FootStep_state_preview object
            %
            %classdef_FootStep_state_preview is a creator.
            %    obj = classdef_FootStep_state_preview(MPC_inputs,xstep,ystep)
            %where MPC_inputs is a classdef_MPC_problem_inputs object,
            %xstep and ystep are the x and y-axis known foot step position
            %
            %Create the COM trajectory matrix based on the properties of
            %MPC_inputs
            
            obj.update_Footstep(MPC_inputs);
            obj.update_initial_state(xstep,ystep);
        end
        %%
        function obj=update_Footstep(obj,MPC_inputs)
            %Create the COM trajectory matrix
            %
            %classdef_FootStep_state_preview/update_Footstep is a function.
            %    [] = update_Footstep(obj,MPC_inputs)
            %where MPC_inputs is a classdef_MPC_problem_inputs object
            %
            %The trajectories are linear with the control parameters
            %such as step = Px_step*step_init + Pu_c*X
            %where X is the associated vector of control parameters (foot step)
            %and step_init is a vector of the known foot step positions
            %at the beginning of the preview window
            %
            %Generate the Px_step and Pu_step matrix
            
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
            %Simplify the COM trajectory matrix with initial COM state
            %
            %classdef_FootStep_state_preview/update_initial_state is a function.
            %    [] = update_initial_state(obj,xstep,ystep)
            %where xstep and ystep are the x and y-axis known foot step position
            %
            %The trajectories are linear with the control parameters
            %such as step = Px_step*step_init + Pu_c*X
            %where X is the associated vector of control parameters (foot step)
            %and step_init is a vector of the known foot step positions
            %at the beginning of the preview window
            %
            %Generate the matrix f_step=Px_step*step_init
            %where each column representing the trajectories along each 
            %axis and the each row is an iteration of the preview window
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