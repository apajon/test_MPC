function [A,B]=compute_coeff_poly6_interpolation(tpassage,psa_zmpinit,psa_zmpfin,nbphases,nbparamABCD)
% Hypo : We consider ZMP trajectory as 5th order polynomials between
% via-points.
% This function compute A and B as:
% C=A*x+B
% Where :
% C are give the polynomials coefficient of ZMP trajectory in one
% direction.
% x are the via-point boundary conditions in position, speed and
% acceleration.
% Initial and final via-point boundary conditions are known and in
% psa_zmpinit and psa_zmpfin
if isempty(psa_zmpinit) && isempty(psa_zmpfin)
    A=zeros(6*nbphases,nbparamABCD);
elseif isempty(psa_zmpinit) || isempty(psa_zmpfin)
    A=zeros(6*nbphases,nbparamABCD+3);
else
    A=zeros(6*nbphases,nbparamABCD+6);
end

for i=1:nbphases
    dt=tpassage(i+1)-tpassage(i);
    m=[1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 2 0 0 0;
       1 dt dt^2 dt^3 dt^4 dt^5;
       0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;
       0 0 2 6*dt 12*dt^2 20*dt^3];
    m=inv(m);
    A(1+(i-1)*6:6+(i-1)*6,:)=[zeros(6,3*(i-1)) m zeros(6,(nbphases-i)*3)];
end

if isempty(psa_zmpinit)
    B=[];
else
    B=A(:,1:3)*psa_zmpinit;
    A(:,1:3)=[];
end
if ~isempty(psa_zmpfin)
    B=A(:,end-2:end)*psa_zmpfin;
    A(:,end-2:end)=[];
end

end