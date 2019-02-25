classdef classdef_quadratic_problem_inputs<handle
    properties
        g %gravity acceleration constant
        omega_temp
        
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
        no_double_support_capture
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
        
        kinematic_limit %COM kinematics limit aka reachable region
        %'' : very simple polyhedron
        %'hexagon' : hexagon kinematic limits
        %'hexagonTranslation' : hexagon kinematic limits with translation
        
        plan_hexagon %plan of kinematic limits polyhedron
        z_leg_min
        z_decalage_tot
        
        OptimCostWeight
        
        right_support% 1 if right foot support otherwise 0 at each sample of the preview window
        left_support% 1 if left foot support otherwise 0 at each sample of the preview window      
        
        xstep
        ystep
        zstep
        
        step_number_pankle_fixed
        %vector of fixed step position on floor [#step x_step y_step; ...]
        %if coordinate is NaN, the position along this axis is still free
        
        zfloor_ref_reduce
        hcom_ref_max_reduce
        
        no_end_constraint

    end
    
    methods
        function obj=classdef_quadratic_problem_inputs()
            %empty constructor
        end
    end
end