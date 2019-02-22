function [Px_c,Pu_c,Px_dc,Pu_dc,Px_ddc,Pu_ddc]=function_compute_com_linear_polyExpo(zeta_ref,T,N)

zeta_k=zeta_ref(1,:);
omega_k=1/sqrt(zeta_k);
T_k=T(1);


eTk=exp(-omega_k*T_k);
Px_c_dc_ddc_k=[1 1/2*(eTk^2-4*eTk+3) 1/2*(eTk-1)^2;...
                0 eTk*(2-eTk) eTk*(1-eTk);...
                0 2*eTk*(eTk-1) eTk*(2*eTk-1)].*repmat([1;omega_k;omega_k^2],1,3).*repmat([1 omega_k^-1 omega_k^-2],3,1);
Px_c_dc_ddc=Px_c_dc_ddc_k;
Pu_c_dc_ddc=(eTk^(-1)-1)*[(eTk-1)^2;...
              (1-eTk)*(2*eTk+1);...
              4*eTk^2+eTk+1].*[1;omega_k;omega_k^2];

for j=2:N
    zeta_k=zeta_ref(j,:);
    omega_k=1/sqrt(zeta_k);
    T_k=T(j);
    
    eTk=exp(-omega_k*T_k);
    Px_c_dc_ddc_k=[Px_c_dc_ddc_k;...
                [1 1/2*(eTk^2-4*eTk+3) 1/2*(eTk-1)^2;...
                0 eTk*(2-eTk) eTk*(1-eTk);...
                0 2*eTk*(eTk-1) eTk*(2*eTk-1)].*repmat([1;omega_k;omega_k^2],1,3).*repmat([1 omega_k^-1 omega_k^-2],3,1)];
    Px_c_dc_ddc=[Px_c_dc_ddc;...
            Px_c_dc_ddc(end-2:end,:)*Px_c_dc_ddc_k(end-2:end,:)];
        
    Pu_c_dc_ddc=[Pu_c_dc_ddc zeros(size(Pu_c_dc_ddc,1),1);...
        zeros(3,size(Pu_c_dc_ddc,2)+1)];
    for k=1:j-1
        Pu_c_dc_ddc(end-2:end,1:end-1)=Px_c_dc_ddc_k(end-2:end,:)*Pu_c_dc_ddc(end-5:end-3,1:end-1);
    end
         %Px_c_k(2:end,:)*tril(ones(size(Pu_c,1))).*diag(Pu_c)' ...
         Pu_c_dc_ddc(end-2:end,end)=(eTk^(-1)-1)*[(eTk-1)^2;...
                                                  (1-eTk)*(2*eTk+1);...
                                                  4*eTk^2+eTk+1].*[1;omega_k;omega_k^2];
end

Px_c=Px_c_dc_ddc(1:3:end,:);
Px_dc=Px_c_dc_ddc(2:3:end,:);
Px_ddc=Px_c_dc_ddc(3:3:end,:);

Pu_c=Pu_c_dc_ddc(1:3:end,:);
Pu_dc=Pu_c_dc_ddc(2:3:end,:);
Pu_ddc=Pu_c_dc_ddc(3:3:end,:);

end