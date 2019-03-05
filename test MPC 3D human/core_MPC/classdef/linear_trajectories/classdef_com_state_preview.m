classdef classdef_com_state_preview<handle
    
    %Class in the MPC core
    %
    %This class create the iterative matrix of COM trajectory at the end of
    %each iteration of the preview window
    %
    %The trajectories are linear with the control parameters
    %such as c = Px_c*c_init + Pu_c*X
    %where X is the associated vector of control parameters
    %and c_init is a vector of the COM [position; velocity; acceleration] 
    %at the beginning of the preview window
    %
    %NOTE the simplification
    %c for COM position
    %dc for COM velocity
    %ddc for COM acceleration
    
    %%
    properties (SetAccess = protected)
        %%
        Px_c %Px matrix of the COM position
        Pu_c %Pu matrix of the COM position
        
        Px_dc %Px matrix of the COM velocity
        Pu_dc %Pu matrix of the COM velocity
        
        Px_ddc %Px matrix of the COM acceleration
        Pu_ddc %Pu matrix of the COM acceleration
        
        %%
        f_c %matrix of f_c=Px_c*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_dc %matrix of f_c=Px_c*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
        
        f_ddc %matrix of f_c=Px_c*c_init where c_init are the COM position, velocity and acceleration at the beginning of the preview window along each direction within each collumn
    end
    %%
    methods
        %%
        function obj=classdef_com_state_preview(MPC_inputs)
            %Creator of the classdef_com_state_preview object
            %
            %classdef_com_state_preview is a creator.
            %    obj = classdef_com_state_preview(MPC_inputs)
            %
            %Create the COM trajectory matrix based on the properties of
            %MPC_inputs
            
            %% Create the COM trajectory matrix
            obj.update_COM_trajectory(MPC_inputs);
            %% Create the COM trajectory matrix based on COM [position; velocity; acceleration] at the beginning of the preview window
            obj.update_initial_state(MPC_inputs);
            
        end
        %%
        function []=update_COM_trajectory(obj,MPC_inputs)
            %Create the COM trajectory matrix
            %
            %classdef_com_state_preview/update_COM_trajectory is a function.
            %    [] = update_COM_trajectory(obj,MPC_inputs)
            %where MPC_inputs is a classdef_MPC_problem_inputs object
            %
            %The trajectories are linear with the control parameters
            %such as c = Px_c*c_init + Pu_c*X
            %where X is the associated vector of control parameters
            %and c_init is a vector of the COM [position; velocity; acceleration] 
            %at the beginning of the preview window
            %
            %Generate the Px_C and Pu_c matrix
            %% compute linear COM and ZMP matrix
            eval(['obj.function_compute_com_linear_' MPC_inputs.COM_form '(MPC_inputs.zeta_temp,MPC_inputs.phase_duration_sampling,MPC_inputs.N)']);

        end
        %%
        function []=update_initial_state(obj,MPC_inputs)
            %Simplify the COM trajectory matrix with initial COM state
            %
            %classdef_com_state_preview/update_initial_state is a function.
            %    [] = update_initial_state(obj,MPC_inputs)
            %where MPC_inputs is a classdef_MPC_problem_inputs object
            %
            %The trajectories are linear with the control parameters
            %such as c = Px_c*c_init + Pu_c*X
            %where X is the associated vector of control parameters
            %and c_init is a vector of the COM [position; velocity; acceleration] 
            %at the beginning of the preview window
            %
            %Generate the matrix of f_c=Px_c*c_init each column representing the
            %trajectories along each axis and the each row is an iteration of the
            %preview window
            
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
        %%
        function []=function_compute_com_linear_comPolynomial(obj,~,T,N)
            %Create the COM trajectory matrix based on polynomial
            %
            %classdef_com_state_preview/function_compute_com_linear_comPolynomial is a function.
            %    [] = function_compute_com_linear_comPolynomial(obj,T,N)
            %where T is a vector of time duration of each iteration of the
            %preview window
            %and N is the number of iteration the preview window
            %
            %COM trajectory interpolation function is a 3rd order
            %polynomial: c=a*t^3+b*t^2+c*t^1+d
            
            T_k=T(1);
            Px_c_dc_ddc_k=[1 T_k  T_k^2/2;...
                            0 1 T_k;...
                            0 0 1];
            Px_c_dc_ddc=Px_c_dc_ddc_k;
            Pu_c_dc_ddc=[T_k^3/6;...
                          T_k^2/2;...
                          T_k];

            for j=2:N
                T_k=T(j);

                Px_c_dc_ddc_k=[Px_c_dc_ddc_k;...
                            [1 T_k  T_k^2/2;...
                              0 1 T_k;...
                              0 0 1]];
                Px_c_dc_ddc=[Px_c_dc_ddc;...
                        Px_c_dc_ddc(end-2:end,:)*Px_c_dc_ddc_k(end-2:end,:)];

                Pu_c_dc_ddc=[Pu_c_dc_ddc zeros(size(Pu_c_dc_ddc,1),1);...
                    zeros(3,size(Pu_c_dc_ddc,2)+1)];
                for k=1:j-1
                    Pu_c_dc_ddc(end-2:end,1:end-1)=Px_c_dc_ddc_k(end-2:end,:)*Pu_c_dc_ddc(end-5:end-3,1:end-1);
                end
                     %Px_c_k(2:end,:)*tril(ones(size(Pu_c,1))).*diag(Pu_c)' ...
                     Pu_c_dc_ddc(end-2:end,end)=[T_k^3/6;...
                                                  T_k^2/2;...
                                                  T_k];
            end

            Px_c=Px_c_dc_ddc(1:3:end,:);
            Px_dc=Px_c_dc_ddc(2:3:end,:);
            Px_ddc=Px_c_dc_ddc(3:3:end,:);

            Pu_c=Pu_c_dc_ddc(1:3:end,:);
            Pu_dc=Pu_c_dc_ddc(2:3:end,:);
            Pu_ddc=Pu_c_dc_ddc(3:3:end,:);
            
            obj.Px_c=Px_c;
            obj.Pu_c=Pu_c;
            obj.Px_dc=Px_dc;
            obj.Pu_dc=Pu_dc;
            obj.Px_ddc=Px_ddc;
            obj.Pu_ddc=Pu_ddc;
        end
        %%
        function []=function_compute_com_linear_comExponential(obj,zeta_ref,T,N)
            %Create the COM trajectory matrix based on exponential
            %
            %classdef_com_state_preview/function_compute_com_linear_comExponential is a function.
            %    [] = function_compute_com_linear_comExponential(obj,zeta_ref,T,N)
            %where T is a vector of time duration of each iteration of the
            %preview window,
            %N is the number of iteration the preview window,
            %and 
            %zeta_ref is a vector of zeta refence of each iteration of the
            %preview window
            %
            %COM trajectory interpolation function is made of exponential:
            %c=a*t+b+c*exp(-w^2*t)+d*exp(w^2*t)
            
            zeta_k=zeta_ref(1,:);
            omega_k=1/sqrt(zeta_k);
            T_k=T(1);

            Px_c_dc_ddc_k=[1 sinh(T_k*omega_k)/omega_k  (cosh(T_k*omega_k)-1)/omega_k^2;...
                            0 cosh(T_k*omega_k) sinh(T_k*omega_k)/omega_k;...
                            0 omega_k*sinh(T_k*omega_k) cosh(T_k*omega_k)];
            Px_c_dc_ddc=Px_c_dc_ddc_k;
            Pu_c_dc_ddc=[-sinh(T_k*omega_k)/omega_k+T_k;...
                          -cosh(T_k*omega_k)+1;...
                          -omega_k*sinh(T_k*omega_k)];

            for j=2:N
                zeta_k=zeta_ref(j,:);
                omega_k=1/sqrt(zeta_k);
                T_k=T(j);

                Px_c_dc_ddc_k=[Px_c_dc_ddc_k;...
                        [1 sinh(T_k*omega_k)/omega_k  (cosh(T_k*omega_k)-1)/omega_k^2;...
                         0 cosh(T_k*omega_k) sinh(T_k*omega_k)/omega_k;...
                         0 omega_k*sinh(T_k*omega_k) cosh(T_k*omega_k)]];
                Px_c_dc_ddc=[Px_c_dc_ddc;...
                        Px_c_dc_ddc(end-2:end,:)*Px_c_dc_ddc_k(end-2:end,:)];

                Pu_c_dc_ddc=[Pu_c_dc_ddc zeros(size(Pu_c_dc_ddc,1),1);...
                    zeros(3,size(Pu_c_dc_ddc,2)+1)];
                for k=1:j-1
                    Pu_c_dc_ddc(end-2:end,1:end-1)=Px_c_dc_ddc_k(end-2:end,:)*Pu_c_dc_ddc(end-5:end-3,1:end-1);
                end
                     %Px_c_k(2:end,:)*tril(ones(size(Pu_c,1))).*diag(Pu_c)' ...
                     Pu_c_dc_ddc(end-2:end,end)=[-sinh(T_k*omega_k)/omega_k+T_k;...
                      -cosh(T_k*omega_k)+1;...
                      -omega_k*sinh(T_k*omega_k)];
            end

            Px_c=Px_c_dc_ddc(1:3:end,:);
            Px_dc=Px_c_dc_ddc(2:3:end,:);
            Px_ddc=Px_c_dc_ddc(3:3:end,:);

            Pu_c=Pu_c_dc_ddc(1:3:end,:);
            Pu_dc=Pu_c_dc_ddc(2:3:end,:);
            Pu_ddc=Pu_c_dc_ddc(3:3:end,:);

            obj.Px_c=Px_c;
            obj.Pu_c=Pu_c;
            obj.Px_dc=Px_dc;
            obj.Pu_dc=Pu_dc;
            obj.Px_ddc=Px_ddc;
            obj.Pu_ddc=Pu_ddc;
        end
        %%
        function []=function_compute_com_linear_comPolyExpo(obj,zeta_ref,T,N)
            %Create the COM trajectory matrix based on polynomial of exponential
            %
            %classdef_com_state_preview/function_compute_com_linear_comPolyExpo is a function.
            %    [] = function_compute_com_linear_comPolyExpo(obj,zeta_ref,T,N)
            %where T is a vector of time duration of each iteration of the
            %preview window,
            %N is the number of iteration the preview window,
            %and 
            %zeta_ref is a vector of zeta refence of each iteration of the
            %preview window
            %
            %COM trajectory interpolation function is made of polynomial of
            %exponential: c=a*exp(w^2*t)+b+c*exp(-*w^2*t)+d*exp(-2*w^2*t)
            
            zeta_k=zeta_ref(1,:);
            omega_k=1/sqrt(zeta_k);
            T_k=T(1);


            eTk=exp(-omega_k*T_k);
            Px_c_dc_ddc_k=[1 1/2*(eTk^2-4*eTk+3) 1/2*(eTk-1)^2;...
                            0 eTk*(2-eTk) eTk*(1-eTk);...
                            0 2*eTk*(eTk-1) eTk*(2*eTk-1)].*repmat([1;omega_k;omega_k^2],1,3).*repmat([1 omega_k^-1 omega_k^-2],3,1);
            Px_c_dc_ddc=Px_c_dc_ddc_k;
            Pu_c_dc_ddc=(eTk^(-1)-1)*[(eTk-1)^2;...
                          (1-eTk)*(2*eTk+1);...
                          4*eTk^2+eTk+1].*[1;omega_k;omega_k^2];

            for j=2:N
                zeta_k=zeta_ref(j,:);
                omega_k=1/sqrt(zeta_k);
                T_k=T(j);

                eTk=exp(-omega_k*T_k);
                Px_c_dc_ddc_k=[Px_c_dc_ddc_k;...
                            [1 1/2*(eTk^2-4*eTk+3) 1/2*(eTk-1)^2;...
                            0 eTk*(2-eTk) eTk*(1-eTk);...
                            0 2*eTk*(eTk-1) eTk*(2*eTk-1)].*repmat([1;omega_k;omega_k^2],1,3).*repmat([1 omega_k^-1 omega_k^-2],3,1)];
                Px_c_dc_ddc=[Px_c_dc_ddc;...
                        Px_c_dc_ddc(end-2:end,:)*Px_c_dc_ddc_k(end-2:end,:)];

                Pu_c_dc_ddc=[Pu_c_dc_ddc zeros(size(Pu_c_dc_ddc,1),1);...
                    zeros(3,size(Pu_c_dc_ddc,2)+1)];
                for k=1:j-1
                    Pu_c_dc_ddc(end-2:end,1:end-1)=Px_c_dc_ddc_k(end-2:end,:)*Pu_c_dc_ddc(end-5:end-3,1:end-1);
                end
                     %Px_c_k(2:end,:)*tril(ones(size(Pu_c,1))).*diag(Pu_c)' ...
                     Pu_c_dc_ddc(end-2:end,end)=(eTk^(-1)-1)*[(eTk-1)^2;...
                                                              (1-eTk)*(2*eTk+1);...
                                                              4*eTk^2+eTk+1].*[1;omega_k;omega_k^2];
            end

            Px_c=Px_c_dc_ddc(1:3:end,:);
            Px_dc=Px_c_dc_ddc(2:3:end,:);
            Px_ddc=Px_c_dc_ddc(3:3:end,:);

            Pu_c=Pu_c_dc_ddc(1:3:end,:);
            Pu_dc=Pu_c_dc_ddc(2:3:end,:);
            Pu_ddc=Pu_c_dc_ddc(3:3:end,:);
            
            obj.Px_c=Px_c;
            obj.Pu_c=Pu_c;
            obj.Px_dc=Px_dc;
            obj.Pu_dc=Pu_dc;
            obj.Px_ddc=Px_ddc;
            obj.Pu_ddc=Pu_ddc;
        end
    end
end