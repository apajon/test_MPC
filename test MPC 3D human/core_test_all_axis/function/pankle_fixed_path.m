function [A,b]=pankle_fixed_path(nbKnownSteps,N,nbVarStep,step_number_pankle_fixed)

A_path=[];
b_path=[];

if size(step_number_pankle_fixed,1)~=0 && nbVarStep~=0 && max(step_number_pankle_fixed(:,1))>(nbKnownSteps-2)
    c = ismember(step_number_pankle_fixed(:,1), nbKnownSteps-2+[1:nbVarStep]');
    indexes = find(c);
    
    for i=1:size(indexes,1)
        M=zeros(1,nbVarStep);
        M(1,step_number_pankle_fixed(indexes(i),1)-nbKnownSteps+2)=1;
        
        %foot step fixed along x-axis
        if ~isnan(step_number_pankle_fixed(indexes(i),2))
            A_path=[A_path;...
                    zeros(1,N) M zeros(1,2*N+nbVarStep)];

            b_path=[b_path;...
                    step_number_pankle_fixed(indexes(i),2)];
        end
            
        %foot step fixed along y-axis
        if ~isnan(step_number_pankle_fixed(indexes(i),3))
            A_path=[A_path;...
                    zeros(1,2*N+nbVarStep) M zeros(1,N)];

            b_path=[b_path;...
                    step_number_pankle_fixed(indexes(i),3)]; 
        end
    end
end

A=A_path;
b=b_path;