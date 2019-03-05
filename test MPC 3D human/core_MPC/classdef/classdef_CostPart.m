classdef classdef_CostPart<handle
    %Class in the MPC core
    %
    %This class create the matrix of the problem cost function depending of
    %each problem variable
    %each iteration of the preview window
    %
    
    %%
    properties (SetAccess = protected)
        %
        H %H matrix of the problem cost of quadprog(H,f)
        
        f %f matrix of the problem cost of quadprog(H,f)

    end
    %%
    methods
        %%
        function obj=classdef_CostPart(Pu1,Pu2,f1,f2)
            %Creator of the classdef_CostPart object
            %
            %classdef_CostPart is a creator.
            %    obj = classdef_CostPart(Pu1,Pu2,f1,f2)
            %
            %Create the Qp problem matrix
            
            obj.update_H(Pu1,Pu2);
            obj.update_f(Pu1,Pu2,f1,f2);
        end
        %%
        function []=update_H(obj,Pu1,Pu2)
            %Create the H matrix
            %
            %classdef_CostPart/update_H is a function.
            %    [] = update_H(obj,Pu1,Pu2)
            %where Pu1 is a Pu matrix related to variables X1
            %and Pu2 is a Pu matrix related to variables X2
            %X1 and X2 must have no variable in common
            %
            %Concatenate the Pu matrix and generate the H matrix
            %compute H as
            %H=[ Pu1'*Pu1 -Pu1*Pu2
            %   -Pu1'*Pu2  Pu2'*Pu2]
            %
            %WARNING
            %if X1 and X2 have variables in common, do the concatenation of
            %Pu1 and Pu2 by your own and use an empty pu2 as input
            
            if isempty(Pu2)
                obj.H=Pu1'*Pu1;
            else
                sym_temp=-Pu1'*Pu2;
                obj.H=[ Pu1'*Pu1    sym_temp;
                      sym_temp.'  Pu2'*Pu2];
            end
        end
        %%
        function []=update_f(obj,Pu1,Pu2,f1,f2)
            %Create the f matrix
            %
            %classdef_CostPart/update_f is a function.
            %    [] = update_f(obj,Pu1,Pu2,f1,f2)
            %where Pu1 is a Pu matrix related to variables X1,
            %Pu2 is a Pu matrix related to variables X2,
            %f1 is a f matrix related to Pu1,
            %and f2 is a f matrix related to Pu2
            %X1 and X2 must have no variable in common
            %
            %Generate the f matrix

            if isempty(Pu2)
                obj.f=Pu1.'*(f1-f2);
            else
                obj.f=[ Pu1.'*(f1-f2);
                       -Pu2.'*(f1-f2)];
            end
        end
    end
end