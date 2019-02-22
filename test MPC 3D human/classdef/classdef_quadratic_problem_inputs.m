classdef classdef_quadratic_problem_inputs<handle
    properties
        N %number of sample of the preview window
        phase_duration_sampling %duration of each sample of the preview window
        phase_type_sampling %phase type of each sample of the preview window
        
        zeta_temp %value of zeta during each sample of the preview window
        zeta_up %value of zeta superior bound during each sample of the preview window
        zeta_down %value of zeta inferior bound during each sample of the preview window
        
        c_init % matrix of CoM initial state with row [c;dc;ddc] and with column along axis [x y z]
        
        dc_ref %reference value of CoM velocity during each sample of the preview window with column along axis [x y]
        
        Px_step %Px matrix of support foot for the preview window
        
        no_double_support
        double_support
        
        phase_type_reduce
        
        backtoankle %from back to ankle of foot
        fronttoankle %from  front to ankle of foot
        exttoankle %from exterior to ankle of foot
        inttoankle %from interior to ankle of foot
        
        sole_margin %sole margin of the support area
        
        xankmax %max ankle stretching distance along x
        xankmin %min ankle stretching distance along x
        yankmax %max ankle stretching distance along y
        yankmin %min ankle stretching distance along y
        
        translate_step_cop_ref %translation from ankle position to CoP reference along axis [x y]
        
        translate_step_polyhedron_type %translation from ankle position to polyhedron of COM reachable region center along axis [x y]
        
        COM_form %COM base trajectory type
        %'com jerk' : COM with piece-wise jerk
        %'zmp vel' : ZMP with piece-wise velocity
        %'poly expo' : 2nd poly of exponential
        
        OptimCostWeight
        
        right_support% 1 if right foot support otherwise 0 at each sample of the preview window
        left_support% 1 if left foot support otherwise 0 at each sample of the preview window      
        
        xstep
        ystep
        zstep
        
        zzmp_ref_reduce

    end
    
    methods
        function obj=classdef_quadratic_problem_inputs()
            %empty constructor
        end
    end
end