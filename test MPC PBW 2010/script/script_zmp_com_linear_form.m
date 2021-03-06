%% ZMP COM Linear form
% ZMP position
%Z_k+1=P_x * x0_k+ P_u * dddX_k
%x0_k=[x(kT);dx(kT);ddx(kT)]
%dddX_k=[dddx_k; ... ;dddx_k+N-1]

Px_z=zeros(N,3);
for i=1:N
    Px_z(i,1:3)=[1 i*T i^2*T^2/2-h_com/g];
end

Pu_z=zeros(N,N);
for i=1:N
    for j=1:i
       Pu_z(i,j)=T^3/6+(i-j)*T^3/2+T*(-h_com/g+1/2*(i-j)^2*T^2); 
    end
end

%% COM Linear form
% COM position
Px_c=zeros(N,3);
for i=1:N
    Px_c(i,1:3)=[1 i*T i^2*T^2/2];
end

Pu_c=zeros(N,N);
for i=1:N
    for j=1:i
       Pu_c(i,j)=T^3/6+(i-j)*T^3/2+(i-j)^2*T^3/2; 
    end
end

% COM velocity
Px_dc=zeros(N,3);
for i=1:N
    Px_dc(i,1:3)=[0 1 i*T];
end

Pu_dc=zeros(N,N);
for i=1:N
    for j=1:i
       Pu_dc(i,j)=T^2/2+(i-j)*T^2; 
    end
end