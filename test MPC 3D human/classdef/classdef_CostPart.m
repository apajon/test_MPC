classdef classdef_CostPart<handle
    %%
    properties (SetAccess = protected)
        %Problem cost matrix of quadprog(H,f)
        H
        
        f

    end
    %%
    methods
        %%
        function obj=classdef_CostPart(Pu1,Pu2,f1,f2)
            %var [var_type1 quantity;var_type2 quantity; ...]
            obj.update_H(Pu1,Pu2);
            obj.update_f(Pu1,Pu2,f1,f2);
        end
        %%
        function []=update_H(obj,Pu1,Pu2)
            %compute H as
            %H=[ Pu1'*Pu1 -Pu1*Pu2
            %   -Pu1'*Pu2  Pu2'*Pu2]
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
            if isempty(Pu2)
                obj.f=Pu1.'*(f1-f2);
            else
                obj.f=[ Pu1.'*(f1-f2);
                       -Pu2.'*(f1-f2)];
            end
        end
    end
end