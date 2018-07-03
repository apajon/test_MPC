function [mdt] = compute_coeff_dddt_matrix(discretization,frequency)
nbphases=length(discretization);
nbpoints=sum(discretization)+1;
mdt=zeros(nbpoints,nbphases*6);
% mdt=[1 0 0 0 0 0 zeros(1,(nbphases-1)*6)];
mdt(1,1:6)=[0 0 2 0 0 0];
nb=0;
rowinitphase=1;
for j=1:nbphases
    rowinitphase=rowinitphase+nb;
    nb=discretization(j);
    for i=1:nb
        dt=i/frequency;%dt is delta(tj)=t-Tj
        Dt=[0 0 2*dt^0 6*dt^1 12*dt^2 20*dt^3];
%         mdt=[mdt;zeros(1,(j-1)*6) Dt zeros(1,(nbphases-j)*6)];
            mdt(rowinitphase+i,(j-1)*6+1:(j-1)*6+6)=Dt;
    end
end
end