classdef classdef_MPC_problem_outputs<handle
    
    %Class in the MPC core
    %The control variable of the MPC scheme are :
    %   COM jerk and horizontal foot step placement
    %This class give the result of each MPC iteration and can also store
    %the results.
    %
    %NOTICE :
    %QP_result_all are build as [X;Y;Z] where X, Y and Z are the vector of
    %control variable along respectively, x, y and z-axis.
    %X is build as [Xc;Xstep] where Xc is a vector of COM control variable
    %along the preview window and Xstep is a vector of foot step position
    %along the preview window
    %Y is build as X
    %Z is only a vector of COM control variable along the preview window
    
    properties
        %%
        QP_result_all %vector of control results of optimization. When results are stored, you can access the i_th stored vector with {i}.

        converge_sampling %optimization convergence result, see exitflag of 'quadprog'
        
        %%
        xc_control %COM control variable of during the next iteration along x-axis
        xc %COM end position of the next iteration along x-axis
        xdc %COM end velocity of the next iteration along x-axis
        xddc %COM end acceleration of the next iteration along x-axis

        yc_control %COM control variable of during the next iteration along y-axis
        yc %COM end position of the next iteration along y-axis
        ydc %COM end velocity of the next iteration along y-axis
        yddc %COM end acceleration of the next iteration along y-axis

        zc_control %COM control variable of during the next iteration along z-axis
        zc %COM end position of the next iteration along z-axis
        zdc %COM end velocity of the next iteration along z-axis
        zddc %COM end acceleration of the next iteration along z-axis

        xstep %foot step position along x-axis
        ystep %foot step position along y-axis
        zstep %foot step position along z-axis (not an optimization variable)
        
%         duration

    end
    
    methods
        function obj=classdef_MPC_problem_outputs()
            %Empty constructor
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/classdef_MPC_problem_outputs is a function.
            %    obj = classdef_MPC_problem_outputs()
        end
        %%
        function obj=update_properties(obj,MPC_inputs,COM_state_preview)
            %Extract the properties value of the next iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/update_properties is a function.
            %    obj = update_properties(obj, MPC_inputs, COM_state_preview)
            %
            %Extract the control value of the next iteration from
            %optimization result
            %
            %Compute the COM end trajectory properties based on the
            %interpolation choice
            %They are all linear function of the control variable of the
            %form : c = f + Pu*X where X is the control variable and are
            %decoupled along each axis
            %
            %Extract the next foot step value if a support foot at the next
            %iteration from optimization result
            

            %% Extract the next COM control variables
            obj.xc_control=obj.QP_result_all(1);
            obj.yc_control=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)/2+1);
            obj.zc_control=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)+1);

            %% Compute the COM end trajectory properties based on the interpolation choice
            obj.xc=COM_state_preview.f_c(1,1)+COM_state_preview.Pu_c(1,1)*obj.xc_control;
            obj.xdc=COM_state_preview.f_dc(1,1)+COM_state_preview.Pu_dc(1,1)*obj.xc_control;
            obj.xddc=COM_state_preview.f_ddc(1,1)+COM_state_preview.Pu_ddc(1,1)*obj.xc_control;

            obj.yc=COM_state_preview.f_c(1,2)+COM_state_preview.Pu_c(1,1)*obj.yc_control;
            obj.ydc=COM_state_preview.f_dc(1,2)+COM_state_preview.Pu_dc(1,1)*obj.yc_control;
            obj.yddc=COM_state_preview.f_ddc(1,2)+COM_state_preview.Pu_ddc(1,1)*obj.yc_control;

            obj.zc=COM_state_preview.f_c(1,3)+COM_state_preview.Pu_c(1,1)*obj.zc_control;
            obj.zdc=COM_state_preview.f_dc(1,3)+COM_state_preview.Pu_dc(1,1)*obj.zc_control;
            obj.zddc=COM_state_preview.f_ddc(1,3)+COM_state_preview.Pu_ddc(1,1)*obj.zc_control;

            %% Extract the next foot step value if it is a support foot at the next iteration
            if (MPC_inputs.phase_type_sampling(1)~='b'&&MPC_inputs.phase_type_sampling(1)~="start"&&MPC_inputs.phase_type_sampling(1)~="stop") && (MPC_inputs.phase_type_sampling(2)=='b'||MPC_inputs.phase_type_sampling(2)=="start"||MPC_inputs.phase_type_sampling(2)=="stop")
                obj.xstep=[obj.xstep;obj.QP_result_all(MPC_inputs.N+1)];
                obj.ystep=[obj.ystep;obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)/2+MPC_inputs.N+1)];
                obj.zstep=[obj.zstep;MPC_inputs.zfloor_ref_reduce(3)];
            end 
        end
        %%
        function obj=add_storage(obj,outputs)
            %Stores the properties value of the next iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/add_storage is a function.
            %    obj = add_storage(obj, outputs)
            %
            %Stores the properties value of 'outputs'
            %'outputs' is a classdef_MPC_problem_outputs object
            %
            %all properties, but QP_result_all, are stored as vector
            %QP_result_all is stored as array of cells accessible with {}
            %instead of ()
            
            obj.QP_result_all{size(obj.QP_result_all,2)+1}=outputs.QP_result_all;
            
            obj.converge_sampling(end+1,:)=outputs.converge_sampling;
            
            obj.xc_control(end+1,:)=outputs.xc_control;
            obj.xc(end+1,:)=outputs.xc;
            obj.xdc(end+1,:)=outputs.xdc;
            obj.xddc(end+1,:)=outputs.xddc;

            obj.yc_control(end+1,:)=outputs.yc_control;
            obj.yc(end+1,:)=outputs.yc;
            obj.ydc(end+1,:)=outputs.ydc;
            obj.yddc(end+1,:)=outputs.yddc;

            obj.zc_control(end+1,:)=outputs.zc_control;
            obj.zc(end+1,:)=outputs.zc;
            obj.zdc(end+1,:)=outputs.zdc;
            obj.zddc(end+1,:)=outputs.zddc;

            if ~isempty(outputs.xstep)
                obj.xstep(end+1,:)=outputs.xstep;
                obj.ystep(end+1,:)=outputs.ystep;
                obj.zstep(end+1,:)=outputs.zstep;
            end
            
        end
        %%
        function obj=MPC_iteration(obj,MPC_inputs,filename)
            %Compute the solution of a MPC iteration
            %
            %CLASSDEF_MPC_PROBLEM_OUTPUTS/MPC_iteration is a function.
            %    obj = MPC_iteration(obj, MPC_inputs, filename)
            %
            %MPC_inputs is a classdef_MPC_problem_inputs object
            %
            %filename is a vector containing the four name of files used
            %to:
            %   -Initialize the matrix for trajectories of the model from 
            %last robot state
            %   -Create the matrix H and f defining the cost function of 
            %the quadratic problem
            %   -Create the matrix A and b defining the inequality
            %   constraints of the quadratic problem
            %   -Create the matrix Aeq and beq defining the equality
            %   constraints of the quadratic problem
            %
            %The solver used is 'quadprog'
            
            %% Check if MPC_inputs is well build
            MPC_inputs.isFull();

            %% Initialization of trajectories from last robot state
%             run('script_initialize_from_current_state.m')
            run(filename(1))

            %% Create cost function
%             run('script_update_cost.m')
            run(filename(2))

            %% Create Inequality constraints
%             run('script_cons_ineq.m')
            run(filename(3))

            %% Create equality Constraints
%             run('script_cons_eq.m')
            run(filename(4))

            %% Unused Constraints
            lb=[];ub=[];x0=[];

            %% Optimization QP options
            %     options=optimoptions('quadprog','Display','iter','MaxIterations',1000);
            options=optimoptions('quadprog','Display','final');
            %     options=optimoptions('quadprog','Display','off');

            %% Optimization QP
            [obj.QP_result_all,tata,obj.converge_sampling]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
            % compute the optimal solution
            
            
            obj.update_properties(MPC_inputs,COM_state_preview);
            %compute the next iteration trajectory based on the optimal control
            
        end
    end
end