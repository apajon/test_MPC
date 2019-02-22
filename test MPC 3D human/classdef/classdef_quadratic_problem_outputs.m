classdef classdef_quadratic_problem_outputs<handle
    properties
        QP_result_all
        converge_sampling
        
        xdddc_storage
        xc
        xdc
        xddc

        ydddc_storage
        yc
        ydc
        yddc

        % COM along z axis
        zdddc_storage
        zc
        zdc
        zddc

        % foot step
        xstep
        ystep
        zstep
        
        duration

    end
    
    methods
        function obj=classdef_quadratic_problem_outputs()
            %empty constructor
        end
        %%
        function obj=update_properties(obj,MPC_inputs,COM_state_preview,COM_form)
            %%
            switch(COM_form)
                case 'com jerk'
                    obj.xdddc_storage=obj.QP_result_all(1);
                    obj.ydddc_storage=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)/2+1);
                    obj.zdddc_storage=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)+1);

                    %% Results COM
                    obj.xc=COM_state_preview.f_c(1,1)+COM_state_preview.Pu_c(1,1)*obj.xdddc_storage;
                    obj.xdc=COM_state_preview.f_dc(1,1)+COM_state_preview.Pu_dc(1,1)*obj.xdddc_storage;
                    obj.xddc=COM_state_preview.f_ddc(1,1)+COM_state_preview.Pu_ddc(1,1)*obj.xdddc_storage;

                    obj.yc=COM_state_preview.f_c(1,2)+COM_state_preview.Pu_c(1,1)*obj.ydddc_storage;
                    obj.ydc=COM_state_preview.f_dc(1,2)+COM_state_preview.Pu_dc(1,1)*obj.ydddc_storage;
                    obj.yddc=COM_state_preview.f_ddc(1,2)+COM_state_preview.Pu_ddc(1,1)*obj.ydddc_storage;

                    obj.zc=COM_state_preview.f_c(1,3)+COM_state_preview.Pu_c(1,1)*obj.zdddc_storage;
                    obj.zdc=COM_state_preview.f_dc(1,3)+COM_state_preview.Pu_dc(1,1)*obj.zdddc_storage;
                    obj.zddc=COM_state_preview.f_ddc(1,3)+COM_state_preview.Pu_ddc(1,1)*obj.zdddc_storage;

                case {'zmp vel', 'poly expo'}
                    obj.xdddc_storage=obj.QP_result_all(1);
                    obj.ydddc_storage=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)/2+1);
                    obj.zdddc_storage=obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)+1);

                    %%
                    obj.xc=COM_state_preview.f_c(1,1)+COM_state_preview.Pu_c(1,1)*obj.xdddc_storage;
                    obj.xdc=COM_state_preview.f_dc(1,1)+COM_state_preview.Pu_dc(1,1)*obj.xdddc_storage;
                    obj.xddc=COM_state_preview.f_ddc(1,1)+COM_state_preview.Pu_ddc(1,1)*obj.xdddc_storage;

                    obj.yc=COM_state_preview.f_c(1,2)+COM_state_preview.Pu_c(1,1)*obj.ydddc_storage;
                    obj.ydc=COM_state_preview.f_dc(1,2)+COM_state_preview.Pu_dc(1,1)*obj.ydddc_storage;
                    obj.yddc=COM_state_preview.f_ddc(1,2)+COM_state_preview.Pu_ddc(1,1)*obj.ydddc_storage;

                    obj.zc=COM_state_preview.f_c(1,3)+COM_state_preview.Pu_c(1,1)*obj.zdddc_storage;
                    obj.zdc=COM_state_preview.f_dc(1,3)+COM_state_preview.Pu_dc(1,1)*obj.zdddc_storage;
                    obj.zddc=COM_state_preview.f_ddc(1,3)+COM_state_preview.Pu_ddc(1,1)*obj.zdddc_storage;

            end    

            if (MPC_inputs.phase_type_sampling(1)~='b'&&MPC_inputs.phase_type_sampling(1)~="start"&&MPC_inputs.phase_type_sampling(1)~="stop") && (MPC_inputs.phase_type_sampling(2)=='b'||MPC_inputs.phase_type_sampling(2)=="start"||MPC_inputs.phase_type_sampling(2)=="stop")
                obj.xstep=[obj.xstep;obj.QP_result_all(MPC_inputs.N+1)];
                obj.ystep=[obj.ystep;obj.QP_result_all((size(obj.QP_result_all,1)-MPC_inputs.N)/2+MPC_inputs.N+1)];
                obj.zstep=[obj.zstep;MPC_inputs.zzmp_ref_reduce(3)];
            end 
        end
        %%
        function obj=add_storage(obj,outputs)
            obj.QP_result_all{size(obj.QP_result_all,2)+1}=outputs.QP_result_all;
            
            obj.converge_sampling(end+1)=outputs.converge_sampling;
            
            obj.xdddc_storage(end+1)=outputs.xdddc_storage;
            obj.xc(end+1)=outputs.xc;
            obj.xdc(end+1)=outputs.xdc;
            obj.xddc(end+1)=outputs.xddc;

            obj.ydddc_storage(end+1)=outputs.ydddc_storage;
            obj.yc(end+1)=outputs.yc;
            obj.ydc(end+1)=outputs.ydc;
            obj.yddc(end+1)=outputs.yddc;

            obj.zdddc_storage(end+1)=outputs.zdddc_storage;
            obj.zc(end+1)=outputs.zc;
            obj.zdc(end+1)=outputs.zdc;
            obj.zddc(end+1)=outputs.zddc;

            if ~isempty(outputs.xstep)
                obj.xstep(end+1)=outputs.xstep;
                obj.ystep(end+1)=outputs.ystep;
                obj.zstep(end+1)=outputs.zstep;
            end
            
        end
    end
end