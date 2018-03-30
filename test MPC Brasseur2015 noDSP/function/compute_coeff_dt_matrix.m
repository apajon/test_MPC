function [mdt] = compute_coeff_dt_matrix(discretization,frequency,nbphases,nbpointdiscret)

mdt=zeros(nbpointdiscret,nbphases*6);
% mdt=[1 0 0 0 0 0 zeros(1,(nbphases-1)*6)];
mdt(1,1:6)=[1 0 0 0 0 0];
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        Dt=[dt^0 dt^1 dt^2 dt^3 dt^4 dt^5];
%         mdt=[mdt;zeros(1,(j-1)*6) Dt zeros(1,(nbphases-j)*6)];
            mdt(rowinitphase+i,(j-1)*6+1:(j-1)*6+6)=Dt;
    end
end
end