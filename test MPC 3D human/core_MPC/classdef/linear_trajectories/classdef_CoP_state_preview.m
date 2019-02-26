classdef classdef_CoP_state_preview<handle
    %%
    properties (SetAccess = protected)
        %linear matrix such as c=Px_c*c_init+Pu_c*X
        %where X is the associated vector of control parameters
        %z for CoP position
        %dz for CoP velocity
        %_up for CoP bound based on zeta_up
        %_down for CoP bound based on zeta_down
        %_mean middle of point between _up and _down
        
        Px_z_up
        Pu_z_up
        
        Px_z_down
        Pu_z_down
        
        Pu_z_mean
        
        Px_dz_up
        Pu_dz_up
        
        Px_dz_down
        Pu_dz_down
        
        Pu_dz_mean
        
        %vector of f_c=Px_c*c_init
        f_z_up
        
        f_z_down
        
        f_z_mean
        
        f_dz_up

        f_dz_down
        
        f_dz_mean
    end
    %%
    methods
        %%
        function obj=classdef_CoP_state_preview(MPC_inputs,COM_state_preview)
            obj.update_LIPM(MPC_inputs,COM_state_preview);
            obj.update_initial_state(MPC_inputs);
        end
        %%
        function []=update_LIPM(obj,MPC_inputs,COM_state_preview)
            %% ZMP up
            obj.Px_z_up=COM_state_preview.Px_c-COM_state_preview.Px_ddc.*repmat(MPC_inputs.zeta_up,1,3);
            obj.Pu_z_up=COM_state_preview.Pu_c-COM_state_preview.Pu_ddc.*repmat(MPC_inputs.zeta_up,1,MPC_inputs.N);

            %% ZMP down
            obj.Px_z_down=COM_state_preview.Px_c-COM_state_preview.Px_ddc.*repmat(MPC_inputs.zeta_down,1,3);
            obj.Pu_z_down=COM_state_preview.Pu_c-COM_state_preview.Pu_ddc.*repmat(MPC_inputs.zeta_down,1,MPC_inputs.N);

            %% ZMP vel
            obj.Px_dz_up=COM_state_preview.Px_dc-eye(size(COM_state_preview.Px_dc)).*repmat(MPC_inputs.zeta_up,1,3);
            obj.Pu_dz_up=COM_state_preview.Pu_dc-eye(size(COM_state_preview.Pu_dc)).*repmat(MPC_inputs.zeta_up,1,MPC_inputs.N);
            
            obj.Px_dz_down=COM_state_preview.Px_dc-eye(size(COM_state_preview.Px_dc)).*repmat(MPC_inputs.zeta_down,1,3);
            obj.Pu_dz_down=COM_state_preview.Pu_dc-eye(size(COM_state_preview.Pu_dc)).*repmat(MPC_inputs.zeta_down,1,MPC_inputs.N);

            %% ZMP mean
            obj.Pu_z_mean=(obj.Pu_z_up+obj.Pu_z_down)/2;
            
            obj.Pu_dz_mean=(obj.Pu_dz_up+obj.Pu_dz_down)/2;

        end
        %%
        function []=update_initial_state(obj,MPC_inputs)
            %% ZMP position up
            obj.f_z_up(:,1)=obj.Px_z_up*MPC_inputs.c_init(:,1);
            obj.f_z_up(:,2)=obj.Px_z_up*MPC_inputs.c_init(:,2);

            %% ZMP position down
            obj.f_z_down(:,1)=obj.Px_z_down*MPC_inputs.c_init(:,1);
            obj.f_z_down(:,2)=obj.Px_z_down*MPC_inputs.c_init(:,2);
            obj.f_z_mean(:,1)=(obj.f_z_up(:,1)+obj.f_z_down(:,1))/2;
            obj.f_z_mean(:,2)=(obj.f_z_up(:,2)+obj.f_z_down(:,2))/2;
            
            %% ZMP mean vel
            obj.f_dz_up(:,1)=obj.Px_dz_up*MPC_inputs.c_init(:,1);
            obj.f_dz_up(:,2)=obj.Px_dz_up*MPC_inputs.c_init(:,2);
            obj.f_dz_down(:,1)=obj.Px_dz_down*MPC_inputs.c_init(:,1);
            obj.f_dz_down(:,2)=obj.Px_dz_down*MPC_inputs.c_init(:,2);

            obj.f_dz_mean(:,1)=(obj.f_dz_up(:,1)+obj.f_dz_down(:,1))/2;
            obj.f_dz_mean(:,2)=(obj.f_dz_up(:,2)+obj.f_dz_down(:,2))/2;
        end
    end
end