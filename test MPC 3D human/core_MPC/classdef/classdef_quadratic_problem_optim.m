classdef classdef_quadratic_problem_optim<handle
    properties
        H
        f
        
        A
        b
        
        Aeq
        beq
        
        lb
        ub
        
        x0
        
        options
    end
    
    methods
        function obj=classdef_quadratic_problem_optim()
            %empty constructor
        end
        function [QP_result_all,converge_sampling]=SolveOptimProblem(obj)
            [QP_result_all,tata,converge_sampling]=quadprog(obj.H,obj.f,obj.A,obj.b,obj.Aeq,obj.beq,obj.lb,obj.ub,obj.x0,obj.options);
        end
    end
end