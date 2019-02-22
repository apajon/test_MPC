classdef classdef_CapturePoint_state_preview<handle
    %%
    properties (SetAccess = protected)
        %linear matrix such as c=Px_c*c_init+Pu_c*X
        %where X is the associated vector of control parameters
        %capture for CapturePoint position
        %_up for CoP bound based on zeta_up
        %_down for CoP bound based on zeta_down
        
        Px_Capture_up
        Pu_Capture_up
        
        Px_Capture_down
        Pu_Capture_down
        
        %vector of f_c=Px_c*c_init
        f_Capture_up
        
        f_Capture_down

    end
    %%
    methods
        %%
        function obj=classdef_CapturePoint_state_preview(MPC_inputs,COM_state_preview)
            obj.update_LIPM(MPC_inputs,COM_state_preview);
            obj.update_initial_state(MPC_inputs);
        end
        %%
        function []=update_LIPM(obj,MPC_inputs,COM_state_preview)
            %% Capture point up
            obj.Px_Capture_up=COM_state_preview.Px_c+COM_state_preview.Px_dc.*repmat(sqrt(MPC_inputs.zeta_up),1,size(COM_state_preview.Px_dc,2));

            obj.Pu_Capture_up=COM_state_preview.Pu_c+COM_state_preview.Pu_dc.*repmat(sqrt(MPC_inputs.zeta_up),1,size(COM_state_preview.Pu_dc,2));

            %% Capture point down
            obj.Px_Capture_down=COM_state_preview.Px_c+COM_state_preview.Px_dc.*repmat(sqrt(MPC_inputs.zeta_down),1,size(COM_state_preview.Px_dc,2));

            obj.Pu_Capture_down=COM_state_preview.Pu_c+COM_state_preview.Pu_dc.*repmat(sqrt(MPC_inputs.zeta_down),1,size(COM_state_preview.Pu_dc,2));


        end
        %%
        function []=update_initial_state(obj,MPC_inputs)
            %% Capture point up
            obj.f_Capture_up(:,1)=obj.Px_Capture_up*MPC_inputs.c_init(:,1);
            obj.f_Capture_up(:,2)=obj.Px_Capture_up*MPC_inputs.c_init(:,2);
            obj.f_Capture_up(:,3)=obj.Px_Capture_up*MPC_inputs.c_init(:,3);

            %% Capture point down
            obj.f_Capture_down(:,1)=obj.Px_Capture_down*MPC_inputs.c_init(:,1);
            obj.f_Capture_down(:,2)=obj.Px_Capture_down*MPC_inputs.c_init(:,2);
            obj.f_Capture_down(:,3)=obj.Px_Capture_down*MPC_inputs.c_init(:,3);

        end
    end
end