function [Px_c,Pu_c,Px_dc,Pu_dc,Px_ddc,Pu_ddc]=function_compute_com_linear_comJerk(T,N)

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

end