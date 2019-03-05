classdef classdef_CoP_state_preview<handle
    %Class in the MPC core
    %
    %This class create the iterative matrix of CoP trajectory at the end of
    %each iteration of the preview window
    %
    %The trajectories are linear with the control parameters
    %such as z = Px_z*c_init + Pu_z*X
    %where X is the associated vector of control parameters
    %and c_init is a vector of the COM [position; velocity; acceleration] 
    %at the beginning of the preview window
    %
    %NOTE the simplification
    %c for COM position
    %dc for COM velocity
    %ddc for COM acceleration
    %
    %z for CoP position
    %dz for CoP velocity
    %
    %_up for CoP bound based on zeta_up
    %_down for CoP bound based on zeta_down
    %_mean middle of point between _up and _down
    %
    %The trajectory are store as matrix with each column representing the
    %trajectories along each axis and the each row is an iteration of the
    %preview window
    %%
    properties (SetAccess = protected)
        %%
        Px_z_up %Px matrix of the CoP_up position
        Pu_z_up %Pu matrix of the CoP_up position
        
        Px_z_down %Px matrix of the CoP_down position
        Pu_z_down %Pu matrix of the CoP_down position
        
        Pu_z_mean %Pu matrix of the CoP_mean position
        
        Px_dz_up %Px matrix of the CoP_up velocity
        Pu_dz_up %Pu matrix of the CoP_up velocity
        
        Px_dz_down %Px matrix of the CoP_down velocity
        Pu_dz_down %Pu matrix of the CoP_down velocity
        
        Pu_dz_mean %Pu matrix of the CoP_mean velocity
        
        %%
        f_z_up %Matrix of f_z_up=Px_z_up*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_z_down %Matrix of f_z_down=Px_z_down*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_z_mean %Matrix of f_z_down=Px_z_down*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_dz_up %Matrix of f_z_up=Px_z_up*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn

        f_dz_down %Matrix of f_z_down=Px_z_down*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_dz_mean %Matrix of f_z_down=Px_z_down*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
    end
    %%
    methods
        %%
        function obj=classdef_CoP_state_preview(MPC_inputs,COM_state_preview)
            %Creator of the classdef_CoP_state_preview object
            %
            %classdef_CoP_state_preview is a creator.
            %    obj = classdef_CoP_state_preview(MPC_inputs)
            %
            %Create the CoP trajectory matrix based on the properties of
            %MPC_inputs and COM_state_preview
            
            %% Create the CoP trajectory matrix
            obj.update_CoP_LIPM(MPC_inputs,COM_state_preview);
            %% Create the CoP trajectory matrix based on COM [position; velocity; acceleration] at the beginning of the preview window
            obj.update_initial_state(MPC_inputs);
        end
        %%
        function []=update_CoP_LIPM(obj,MPC_inputs,COM_state_preview)
            %Create the CoP trajectory matrix
            %
            %classdef_cop_state_preview/update_CoP_LIPM is a function.
            %    [] = update_CoP_LIPM(obj,MPC_inputs,COM_state_preview)
            %where MPC_inputs is a classdef_MPC_problem_inputs object
            %and COM_state_preview is a classdef_com_state_preview object
            %
            %The trajectories are linear with the control parameters
            %such as c = Px_c*c_init + Pu_c*X
            %where X is the associated vector of control parameters
            %and c_init is a vector of the COM [position; velocity; acceleration] 
            %at the beginning of the preview window
            %
            %Based on the LIPM, the CoP equation is
            %z=c-zeta*ddc
            %dz=dc-zeta*dddc
            
            %% CoP up
            obj.Px_z_up=COM_state_preview.Px_c-COM_state_preview.Px_ddc.*repmat(MPC_inputs.zeta_up,1,3);
            obj.Pu_z_up=COM_state_preview.Pu_c-COM_state_preview.Pu_ddc.*repmat(MPC_inputs.zeta_up,1,MPC_inputs.N);

            %% CoP down
            obj.Px_z_down=COM_state_preview.Px_c-COM_state_preview.Px_ddc.*repmat(MPC_inputs.zeta_down,1,3);
            obj.Pu_z_down=COM_state_preview.Pu_c-COM_state_preview.Pu_ddc.*repmat(MPC_inputs.zeta_down,1,MPC_inputs.N);

            %% CoP vel up
            obj.Px_dz_up=COM_state_preview.Px_dc-eye(size(COM_state_preview.Px_dc)).*repmat(MPC_inputs.zeta_up,1,3);
            obj.Pu_dz_up=COM_state_preview.Pu_dc-eye(size(COM_state_preview.Pu_dc)).*repmat(MPC_inputs.zeta_up,1,MPC_inputs.N);
            
            %% CoP vel down
            obj.Px_dz_down=COM_state_preview.Px_dc-eye(size(COM_state_preview.Px_dc)).*repmat(MPC_inputs.zeta_down,1,3);
            obj.Pu_dz_down=COM_state_preview.Pu_dc-eye(size(COM_state_preview.Pu_dc)).*repmat(MPC_inputs.zeta_down,1,MPC_inputs.N);

            %% CoP mean
            obj.Pu_z_mean=(obj.Pu_z_up+obj.Pu_z_down)/2;
            obj.Pu_dz_mean=(obj.Pu_dz_up+obj.Pu_dz_down)/2;

        end
        %%
        function []=update_initial_state(obj,MPC_inputs)
            %Simplify the CoP trajectory matrix with initial COM state
            %
            %classdef_com_state_preview/update_initial_state is a function.
            %    [] = update_initial_state(obj,MPC_inputs)
            %where MPC_inputs is a classdef_MPC_problem_inputs object
            %
            %The trajectories are linear with the control parameters
            %such as z = Px_z*c_init + Pu_z*X
            %where X is the associated vector of control parameters
            %and c_init is a vector of the COM [position; velocity; acceleration] 
            %at the beginning of the preview window
            %
            %Generate the matrix of f_z=Pz_c*c_init each column representing the
            %trajectories along each axis and the each row is an iteration of the
            %preview window
            
            %% CoP position up
            obj.f_z_up(:,1)=obj.Px_z_up*MPC_inputs.c_init(:,1);
            obj.f_z_up(:,2)=obj.Px_z_up*MPC_inputs.c_init(:,2);

            %% CoP position down
            obj.f_z_down(:,1)=obj.Px_z_down*MPC_inputs.c_init(:,1);
            obj.f_z_down(:,2)=obj.Px_z_down*MPC_inputs.c_init(:,2);
            
            %% CoP position mean
            obj.f_z_mean(:,1)=(obj.f_z_up(:,1)+obj.f_z_down(:,1))/2;
            obj.f_z_mean(:,2)=(obj.f_z_up(:,2)+obj.f_z_down(:,2))/2;
            
            %% CoP mean vel
            obj.f_dz_up(:,1)=obj.Px_dz_up*MPC_inputs.c_init(:,1);
            obj.f_dz_up(:,2)=obj.Px_dz_up*MPC_inputs.c_init(:,2);
            obj.f_dz_down(:,1)=obj.Px_dz_down*MPC_inputs.c_init(:,1);
            obj.f_dz_down(:,2)=obj.Px_dz_down*MPC_inputs.c_init(:,2);

            obj.f_dz_mean(:,1)=(obj.f_dz_up(:,1)+obj.f_dz_down(:,1))/2;
            obj.f_dz_mean(:,2)=(obj.f_dz_up(:,2)+obj.f_dz_down(:,2))/2;
        end
    end
end