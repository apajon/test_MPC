classdef classdef_create_experiment<handle
    %%
    properties
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
        
        preview_windows_duration
        
        phase_duration
        phase_duration_iteration
        phase_duration_cumul
        phase_duration_iteration_cumul
        
        T_r
        T_l
        T_b
        T_start
        T_stop
        
        phase_type
        
        phase_type_sampling

    end
    %%
    methods
        %%
        function obj=classdef_create_experiment(phase_duration_type,nb_foot_step,firstSS)
            obj.generate_phase_type(nb_foot_step,firstSS);
            
            obj.generate_phase_duration(phase_duration_type);
            
            obj.generate_phase_type_sampling();
            
        end
        %%
        function obj=generate_phase_duration(obj,phase_duration_type)
            %% Phase duration
            phase_duration_type_namefile=['phase_duration/' phase_duration_type '.m'];
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
        %% 
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
        function obj=generate_phase_type_sampling(obj)
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
        end
    end
end