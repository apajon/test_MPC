classdef classdef_create_experiment<handle
    %%
    properties
        %%
        g
        omega_temp
        zeta_temp
        
        %%
        phase_duration_r
        phase_duration_l
        phase_duration_b
        phase_duration_start
        phase_duration_stop

        N_r
        N_l
        N_b
        N_start
        N_stop
        
        %%
        preview_windows_duration
        
        phase_duration
        phase_duration_iteration
        phase_duration_cumul
        phase_duration_iteration_cumul
        
        %%
        T_r
        T_l
        T_b
        T_start
        T_stop
        
        %%
        phase_type
        
        phase_type_sampling
        phase_duration_sampling
        phase_duration_sampling_cumul
        
        phase_sampling_length
        phase_type_decouple
        
        %%
        yaw
        
        yaw_sampling
        
        %%
        Px_step_ref
        
        plan_hexagon
        z_leg_min
        z_decalage_tot
        
        translate_step_polyhedron_type
        
        OptimCostWeight
        
        step_number_pankle_fixed
        
        vcom_ref
        vcom_change
        vcom_1
        vcom_2
        
        zfloor_ref
        
        hcom_ref
        hcom_ref_max
        
        zeta_up_ref
        zeta_down_ref
        
        zstep_l_0
        zstep_r_0
        
        zstep_l_ref
        zstep_r_ref

    end
    %%
    methods
        %%
        function obj=classdef_create_experiment()
            %empty creator
        end
        %%
        function obj=classdef_create_experiment_test(obj,phase_duration_type,...
                nb_foot_step,firstSS,...
                kinematic_limit,robot,polyhedron_position,...
                optimweight_filename,...
                walking_type)
            %% Mechanics
            obj.g=9.81; %m.s-1
            obj.omega_temp=sqrt(obj.g/robot.h_com);
            obj.zeta_temp=1/obj.omega_temp^2;
            
            %% phase duration
            obj.generate_phase_type(nb_foot_step,firstSS);
            
            obj.generate_phase_duration(phase_duration_type);
            
            obj.generate_phase_sampling();
            
            obj.generate_phase_decoupling(firstSS);
            
            %% ref
            obj.generate_kinematic_limit(kinematic_limit,robot,polyhedron_position);
            
            obj.generate_optimCostWeight(optimweight_filename);
            
            obj.generate_ref(walking_type,robot);
        end
        %%
        function obj=generate_phase_duration(obj,phase_duration_type)
            %% Phase duration
            phase_duration_type_namefile=['config/phase_duration/' phase_duration_type '.m'];
            if isfile(phase_duration_type_namefile)
                run(phase_duration_type_namefile)
            else
                msg='Bad choice of phase_duration_type \n';
                errormsg=[msg];
                error(errormsg,[])  
            end

            obj.phase_duration_r=phase_duration_r;
            obj.phase_duration_l=phase_duration_l;
            obj.phase_duration_b=phase_duration_b;
            obj.phase_duration_start=phase_duration_start;
            obj.phase_duration_stop=phase_duration_stop;

            obj.N_r=N_r;
            obj.N_l=N_l;
            obj.N_b=N_b;
            obj.N_start=N_start;
            obj.N_stop=N_stop;
            
            obj.preview_windows_duration=phase_duration_r+phase_duration_l+phase_duration_b*2;

            obj.T_r=phase_duration_r/N_r;
            obj.T_l=phase_duration_l/N_l;
            obj.T_b=phase_duration_b/N_b;
            obj.T_start=phase_duration_start/N_start;
            obj.T_stop=phase_duration_stop/N_stop;
            %T=0.1;
            
            %% Phase duration definition
            phase_duration_=zeros(length(obj.phase_type),1);
            phase_duration_(any(obj.phase_type=='r',2))=obj.phase_duration_r;
            phase_duration_(any(obj.phase_type=='l',2))=obj.phase_duration_l;
            phase_duration_(any(obj.phase_type=='b',2))=obj.phase_duration_b;
            phase_duration_(any(obj.phase_type=='start',2))=obj.phase_duration_start;
            phase_duration_(any(obj.phase_type=='stop',2))=obj.phase_duration_stop;
            
            obj.phase_duration=phase_duration_;

            phase_duration_iteration_=zeros(length(obj.phase_type),1);
            phase_duration_iteration_(any(obj.phase_type=='r',2))=obj.N_r;
            phase_duration_iteration_(any(obj.phase_type=='l',2))=obj.N_l;
            phase_duration_iteration_(any(obj.phase_type=='b',2))=obj.N_b;
            phase_duration_iteration_(any(obj.phase_type=='start',2))=obj.N_start-1;
            phase_duration_iteration_(any(obj.phase_type=='stop',2))=obj.N_stop;

            obj.phase_duration_iteration=phase_duration_iteration_;

            %% Phase duration cumulative
            obj.phase_duration_cumul=cumsum(phase_duration_);
            obj.phase_duration_iteration_cumul=cumsum(phase_duration_iteration_);
        end 
        function obj=generate_phase_type(obj,nb_foot_step,firstSS)
            %%
            phase_type_="start";
            for k=1:nb_foot_step
                if firstSS=='r'
                    if mod(k,2)
                        phase_type_=[phase_type_;'r';'b'];
                    else
                        phase_type_=[phase_type_;'l';'b'];
                    end
                elseif firstSS=='l'
                    if mod(k,2)
                        phase_type_=[phase_type_;'l';'b'];
                    else
                        phase_type_=[phase_type_;'r';'b'];
                    end
                else
                    msg='Bad choice of firstSS \n';
                    errormsg=[msg];
                    error(errormsg,[])  
                end
            end
            phase_type_(end,:)="stop";
            
            obj.phase_type=phase_type_;
        end
        function obj=generate_phase_sampling(obj)
            %% Phase type sampling
            phase_type_sampling_=[];
            for i=1:length(obj.phase_duration)
                switch(obj.phase_type(i))
                    case 'start'
                        phase_type_sampling_temp=repmat(obj.phase_type(i),obj.N_start-1,1);
                    case 'stop'
                        phase_type_sampling_temp=repmat(obj.phase_type(i),obj.N_stop,1);
                    case 'r'
                        phase_type_sampling_temp=repmat(obj.phase_type(i),obj.N_r,1);
                    case 'l'
                        phase_type_sampling_temp=repmat(obj.phase_type(i),obj.N_l,1);
                    case 'b'
                        phase_type_sampling_temp=repmat(obj.phase_type(i),obj.N_b,1);
                    otherwise
                        error(['Unknown phase type of phase_type(i) with i = ' num2str(i)])
                end
                phase_type_sampling_=[phase_type_sampling_;phase_type_sampling_temp];        
            end

            %add phase type sampling for the last preview windows
            switch(obj.phase_type(end))
                case 'start'
                    phase_type_sampling_temp=repmat(obj.phase_type(end),ceil(obj.preview_windows_duration/obj.T_start),1);
                case 'stop'
                    phase_type_sampling_temp=repmat(obj.phase_type(end),ceil(obj.preview_windows_duration/obj.T_stop),1);
                case 'r'
                    phase_type_sampling_temp=repmat(obj.phase_type(end),ceil(obj.preview_windows_duration/obj.T_r),1);
                case 'l'
                    phase_type_sampling_temp=repmat(obj.phase_type(end),ceil(obj.preview_windows_duration/obj.T_l),1);
                case 'b'
                    phase_type_sampling_temp=repmat(obj.phase_type(end),ceil(obj.preview_windows_duration/obj.T_b),1);
                otherwise
                    error(['Unknown phase type of phase_type(i) with i = ' num2str(i)])
            end
            phase_type_sampling_=[phase_type_sampling_;phase_type_sampling_temp];
            
            obj.phase_type_sampling=phase_type_sampling_;
            
            %% Phase sampling duration
            phase_duration_sampling_=zeros(length(obj.phase_type_sampling),1);
            phase_duration_sampling_(any(obj.phase_type_sampling=="start",2))=obj.T_start;
            phase_duration_sampling_(any(obj.phase_type_sampling=="stop",2))=obj.T_stop;
            phase_duration_sampling_(any(obj.phase_type_sampling=='r',2))=obj.T_l;
            phase_duration_sampling_(any(obj.phase_type_sampling=='l',2))=obj.T_l;
            phase_duration_sampling_(any(obj.phase_type_sampling=='b',2))=obj.T_b;

            phase_duration_sampling_=[obj.T_start;phase_duration_sampling_(1:end-1,:)];

            if sum(any(phase_duration_sampling_==0,2))~=0
                error(['Undefined phase type in phase_type_sampling'])
            end
            
            obj.phase_duration_sampling=phase_duration_sampling_;

            obj.phase_duration_sampling_cumul=cumsum(phase_duration_sampling_);

            % phase_type_sampling=[phase_type_sampling(2:end);phase_type_sampling(end)];
        end
        function obj=generate_phase_decoupling(obj,firstSS)
            %% phase decoupling
            obj.phase_sampling_length=[1;cumsum(obj.phase_duration_iteration(1:end-1))+1];
            % obj.phase_sampling_length=[1;cumsum(phase_duration_iteration(1:end-1))];

            %%
            phase_type_decouple_=zeros(length(obj.phase_type_sampling),length(obj.phase_type));
            for i=1:length(obj.phase_sampling_length)-1
                phase_type_decouple_(:,i)=[zeros(obj.phase_sampling_length(i)-1,1);ones(obj.phase_sampling_length(i+1)-obj.phase_sampling_length(i),1);zeros(length(obj.phase_type_sampling)-obj.phase_sampling_length(i+1)+1,1)];
            end
            phase_type_decouple_(:,end)=[zeros(obj.phase_sampling_length(end)-1,1);ones(length(obj.phase_type_sampling)-obj.phase_sampling_length(end)+1,1)];

            phase_type_decouple_(:,1:2:end)=phase_type_decouple_(:,1:2:end)./2;
            
            obj.phase_type_decouple=phase_type_decouple_;
            

            Px_step_rl=phase_type_decouple_(:,2:2:end);
            Px_step_b=phase_type_decouple_(:,1:2:end);

            for i=2:size(Px_step_b,2)
                Px_step_rl(find(Px_step_b(:,i),1),i-1)=1;
                Px_step_b(find(Px_step_b(:,i),1),i)=0;
            end

            Px_step_ref_=[Px_step_b zeros(size(Px_step_b,1),1)]+[zeros(size(Px_step_b,1),1) Px_step_b];
            if obj.phase_type(end)=='stop' ||  obj.phase_type(end)=='b'
                Px_step_ref_(:,2:end-1)=Px_step_ref_(:,2:end-1)+Px_step_rl;
            else
                Px_step_ref_(:,2:end)=Px_step_ref_(:,2:end)+Px_step_rl;
            end
            
            obj.Px_step_ref=Px_step_ref_;
            
            yaw_=0;
            for k=3:size(Px_step_ref_,2)-2*(obj.phase_type(end)=='stop')
                if firstSS=='r'
                    yaw_(k-1,1)=(-1)^mod(k+1,2)*0.1745;
                elseif firstSS=='l'
                    yaw_(k-1,1)=(-1)^mod(k-1,2)*0.1745;
                else
                    error('Bad choice of firstSS')
                end
            end
            if obj.phase_type(end)=='stop'
                yaw_(end+1)=0;
            end
            obj.yaw=yaw_;
            
            switch obj.phase_type(end)
                case 'stop'
                    yaw_sampling_=(Px_step_ref_(:,2:end-1)~=0)*yaw_;
                otherwise
                    yaw_sampling_=(Px_step_ref_(:,2:end)~=0)*yaw_;
            end
            obj.yaw_sampling=yaw_sampling_;
        end
        function obj=generate_kinematic_limit(obj,kinematic_limit,robot,polyhedron_position)
            %% Polyhedron limits
            switch kinematic_limit
                case ''
                    number_level=[];
                    run('script_polyhedron.m')
                case 'hexagon'
                    number_level=2;
                    run('script_polyhedron_hexagon.m')
                case 'hexagonTranslation'
                    number_level=2;
                    run('script_polyhedron_hexagon_translation.m')
                otherwise
                    error('choose a type of kinematic_limit')
            end
            
            obj.plan_hexagon=plan_hexagon;
            obj.z_leg_min=z_leg_min;
            obj.z_decalage_tot=z_decalage_tot;
            
            switch(polyhedron_position)
                case 'ankle_center'
                     obj.translate_step_polyhedron_type=[0 0];
                case 'foot_center'
                     obj.translate_step_polyhedron_type=[(fronttoankle+backtoankle)/2-backtoankle ...
                         -((exttoankle+inttoankle)/2-inttoankle)];
                case 'waist_center'
                     obj.translate_step_polyhedron_type=[0 0.06845];
                 otherwise
                    msg='Bad choice of polyhedron_type \n';
                    msg1='ankle_center \n';
                    msg2='foot_center \n';
                    msg3='waist_center \n';
                    errormsg=[msg msg1 msg2 msg3];
                    error(errormsg,[])
            end
        end
        function obj=generate_optimCostWeight(obj,optimweight_filename)
            run(optimweight_filename)
            
            obj.OptimCostWeight=OptimCostWeight;
        end
        function obj=generate_ref(obj,walking_type,robot)
            %% fixed step position after initial robot step state position
            step_number_pankle_fixed=[];

            switch(walking_type)
                case 4
                run('pankle_fixed/pankle_fixed_04.m')
                case 5
                run('pankle_fixed/pankle_fixed_05.m')
            end
            
            obj.step_number_pankle_fixed=step_number_pankle_fixed;
            %% COM vel ref
            xvcom_ref_=zeros(obj.phase_duration_iteration_cumul(end),1);
            yvcom_ref_=zeros(obj.phase_duration_iteration_cumul(end),1);

            % vcom_change=round(size(xvcom_ref,1)*1/2)+4;
            vcom_change_=round(size(xvcom_ref_,1)*1/3)+16;

            switch(walking_type)
                case 1
                    vcom_1_=0.315; %m.s^-1
                    vcom_2_=0.315; %m.s^-1
            %         vcom_1=0.6; %m.s^-1
            %         vcom_2=0.6; %m.s^-1
                case 2
                    vcom_1_=0.315; %m.s^-1
                    vcom_2_=0.315; %m.s^-1
                case 3
                    vcom_1_=0.6; %m.s^-1
                    vcom_2_=1.2; %m.s^-1
                case 4
                    vcom_1_=0; %m.s^-1
                    vcom_2_=0; %m.s^-1
                case 5
                    vcom_1_=0; %m.s^-1
                    vcom_2_=0; %m.s^-1
            end
            xvcom_ref_(obj.N_start+1:vcom_change_)=xvcom_ref_(obj.N_start+1:vcom_change_)+vcom_1_; %m.s^-1
            xvcom_ref_(vcom_change_+1:end-obj.N_stop)=xvcom_ref_(vcom_change_+1:end-obj.N_stop)+vcom_2_; %m.s^-1

            xvcom_ref_=[xvcom_ref_;zeros(size(obj.phase_type_sampling,1)-size(xvcom_ref_,1),1)];
            yvcom_ref_=[yvcom_ref_;zeros(size(obj.phase_type_sampling,1)-size(yvcom_ref_,1),1)];

%             obj.xvcom_ref=xvcom_ref_;
%             obj.yvcom_ref=yvcom_ref_;

            obj.vcom_ref=[xvcom_ref_ yvcom_ref_];
            obj.vcom_change=vcom_change_;
            obj.vcom_1=vcom_1_;
            obj.vcom_2=vcom_2_;

            %% ZMP height ref
            % zzmp_ref=zeros(round(max(phase_duration_cumul)/T+N),1);
            % zzmp_ref(round(phase_duration_cumul(3)/T)+1:round(phase_duration_cumul(5)/T))=0.5;
            % zzmp_ref(round(phase_duration_cumul(5)/T)+1:round(phase_duration_cumul(7)/T))=0.1;
            % zzmp_ref(round(phase_duration_cumul(7)/T)+1:round(phase_duration_cumul(9)/T))=0.15;
            % zzmp_ref(round(phase_duration_cumul(9)/T)+1:end)=0.2;

            zzmp_ref_=zeros(size(xvcom_ref_,1),1);
            switch(walking_type)
                case 1
                case 2
                    zzmp_ref_(obj.phase_duration_iteration_cumul(5)+1:obj.phase_duration_iteration_cumul(7))=0.195+0.18*0;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(7)+1:obj.phase_duration_iteration_cumul(9))=0.195+0.18*1;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(9)+1:obj.phase_duration_iteration_cumul(11))=0.195+0.18*2;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(11)+1:obj.phase_duration_iteration_cumul(13))=0.195+0.18*3;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(13)+1:end)=0.195+0.18*3+0.145;

            %         zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=-0.195-0.18*0;
            %         zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=-0.195-0.18*1;
            %         zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=-0.195-0.18*2;
            %         zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=-0.195-0.18*3;
            %         zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=-0.195-0.18*3-0.145;



                case 3
                case 4
                case 5
                    zzmp_ref_(obj.phase_duration_iteration_cumul(5)+1:obj.phase_duration_iteration_cumul(7))=0.195+0.18*0;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(7)+1:obj.phase_duration_iteration_cumul(9))=0.195+0.18*1;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(9)+1:obj.phase_duration_iteration_cumul(11))=0.195+0.18*2;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(11)+1:obj.phase_duration_iteration_cumul(13))=0.195+0.18*3;
                    zzmp_ref_(obj.phase_duration_iteration_cumul(13)+1:end)=0.195+0.18*3+0.145;

            %         zzmp_ref(phase_duration_iteration_cumul(5)+1:phase_duration_iteration_cumul(7))=-0.195-0.18*0;
            %         zzmp_ref(phase_duration_iteration_cumul(7)+1:phase_duration_iteration_cumul(9))=-0.195-0.18*1;
            %         zzmp_ref(phase_duration_iteration_cumul(9)+1:phase_duration_iteration_cumul(11))=-0.195-0.18*2;
            %         zzmp_ref(phase_duration_iteration_cumul(11)+1:phase_duration_iteration_cumul(13))=-0.195-0.18*3;
            %         zzmp_ref(phase_duration_iteration_cumul(13)+1:end)=-0.195-0.18*3-0.145;
            end
            
            obj.zfloor_ref=zzmp_ref_;


            %% COM height ref
            hcom_ref_=zeros(size(xvcom_ref_,1),1);

            hcom_ref_=hcom_ref_+robot.h_com+zzmp_ref_;
            
            obj.hcom_ref=hcom_ref_;

            hcom_ref_max_=hcom_ref_;
            hcom_ref_max_(11:length(xvcom_ref_)-40)=hcom_ref_max_(11:length(xvcom_ref_)-40)+robot.h_com_max+0.2;
            
            obj.hcom_ref_max=hcom_ref_max_;

            %% zeta boundaries ref
            obj.zeta_up_ref=(hcom_ref_+robot.h_com_max-zzmp_ref_+0.32)/obj.g;

            obj.zeta_down_ref=(hcom_ref_+robot.h_com_min-zzmp_ref_-0.02)/obj.g;

            %% foot step height ref
            obj.zstep_l_0=zzmp_ref_(1);
            obj.zstep_r_0=zzmp_ref_(1);

            obj.zstep_r_ref=zzmp_ref_;

            obj.zstep_l_ref=zzmp_ref_;
        end
    end
end