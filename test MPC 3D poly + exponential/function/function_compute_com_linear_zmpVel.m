function [Px_c,Pu_c,Px_dc,Pu_dc,Px_ddc,Pu_ddc]=function_compute_com_linear_zmpVel(zeta_ref,T,N)

zeta_k=zeta_ref(1,:);
omega_k=1/sqrt(zeta_k);

Px_c_dc_ddc_k=[1 sinh(T*omega_k)/omega_k  (cosh(T*omega_k)-1)/omega_k^2;...
                0 cosh(T*omega_k) sinh(T*omega_k)/omega_k;...
                0 omega_k*sinh(T*omega_k) cosh(T*omega_k)];
Px_c_dc_ddc=Px_c_dc_ddc_k;
Pu_c_dc_ddc=[-sinh(T*omega_k)/omega_k+T;...
              -cosh(T*omega_k)+1;...
              -omega_k*sinh(T*omega_k)];

for j=2:N
    zeta_k=zeta_ref(j,:);
    omega_k=1/sqrt(zeta_k);
    Px_c_dc_ddc_k=[Px_c_dc_ddc_k;...
            [1 sinh(T*omega_k)/omega_k  (cosh(T*omega_k)-1)/omega_k^2;...
             0 cosh(T*omega_k) sinh(T*omega_k)/omega_k;...
             0 omega_k*sinh(T*omega_k) cosh(T*omega_k)]];
    Px_c_dc_ddc=[Px_c_dc_ddc;...
            Px_c_dc_ddc(end-2:end,:)*Px_c_dc_ddc_k(end-2:end,:)];
        
    Pu_c_dc_ddc=[Pu_c_dc_ddc zeros(size(Pu_c_dc_ddc,1),1);...
        zeros(3,size(Pu_c_dc_ddc,2)+1)];
    for k=1:j-1
        Pu_c_dc_ddc(end-2:end,1:end-1)=Px_c_dc_ddc_k(end-2:end,:)*Pu_c_dc_ddc(end-5:end-3,1:end-1);
    end
         %Px_c_k(2:end,:)*tril(ones(size(Pu_c,1))).*diag(Pu_c)' ...
         Pu_c_dc_ddc(end-2:end,end)=[-sinh(T*omega_k)/omega_k+T;...
          -cosh(T*omega_k)+1;...
          -omega_k*sinh(T*omega_k)];
end

Px_c=Px_c_dc_ddc(1:3:end,:);
Px_dc=Px_c_dc_ddc(2:3:end,:);
Px_ddc=Px_c_dc_ddc(3:3:end,:);

Pu_c=Pu_c_dc_ddc(1:3:end,:);
Pu_dc=Pu_c_dc_ddc(2:3:end,:);
Pu_ddc=Pu_c_dc_ddc(3:3:end,:);

end