classdef classdef_com_state_preview<handle
    %%
    properties (SetAccess = protected)
        %linear matrix such as c=Px_c*c_init+Pu_c*X
        %where X is the associated vector of control parameters
        %c for COM position
        %dc for COM velocity
        %ddc for COM acceleration
        Px_c
        Pu_c
        
        Px_dc
        Pu_dc
        
        Px_ddc
        Pu_ddc
        
        %vector of f_c=Px_c*c_init
        f_c
        
        f_dc
        
        f_ddc
    end
    %%
    methods
        %%
        function obj=classdef_com_state_preview(MPC_inputs)
            obj.update_COM_trajectory(MPC_inputs);
            obj.update_initial_state(MPC_inputs);
            
        end
        %%
        function []=update_COM_trajectory(obj,MPC_inputs)
            %% compute linear COM and ZMP matrix
            switch(MPC_inputs.COM_form)
                case 'com jerk'
                    [obj.Px_c,obj.Pu_c,obj.Px_dc,obj.Pu_dc,obj.Px_ddc,obj.Pu_ddc]=function_compute_com_linear_comJerk(MPC_inputs.phase_duration_sampling,MPC_inputs.N);
                case 'zmp vel'
                    [obj.Px_c,obj.Pu_c,obj.Px_dc,obj.Pu_dc,obj.Px_ddc,obj.Pu_ddc]=function_compute_com_linear_zmpVel(MPC_inputs.zeta_temp,MPC_inputs.phase_duration_sampling,MPC_inputs.N);
                case 'poly expo'
                    [obj.Px_c,obj.Pu_c,obj.Px_dc,obj.Pu_dc,obj.Px_ddc,obj.Pu_ddc]=function_compute_com_linear_polyExpo(MPC_inputs.zeta_temp,MPC_inputs.phase_duration_sampling,MPC_inputs.N);
                otherwise
                    error('Bad COM_form')
            end
        end
        %%
        function []=update_initial_state(obj,MPC_inputs)
            %% initialization based on initial COM state from MPC_inputs
            %% COM position
            obj.f_c(:,1)=obj.Px_c*MPC_inputs.c_init(:,1);
            obj.f_c(:,2)=obj.Px_c*MPC_inputs.c_init(:,2);
            obj.f_c(:,3)=obj.Px_c*MPC_inputs.c_init(:,3);
            %% COM velocity
            obj.f_dc(:,1)=obj.Px_dc*MPC_inputs.c_init(:,1);
            obj.f_dc(:,2)=obj.Px_dc*MPC_inputs.c_init(:,2);
            obj.f_dc(:,3)=obj.Px_dc*MPC_inputs.c_init(:,3);

            %% COM acceleration
            obj.f_ddc(:,1)=obj.Px_ddc*MPC_inputs.c_init(:,1);
            obj.f_ddc(:,2)=obj.Px_ddc*MPC_inputs.c_init(:,2);
            obj.f_ddc(:,3)=obj.Px_ddc*MPC_inputs.c_init(:,3);
        end
    end
end