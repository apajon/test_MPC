function [delta] = c(i,j,P,Q)
%EITER, Thomas et MANNILA,Heikki. 
% "Computing discrete Fréchet distance."
% Tech. Report CD-TR 94/64, 
% Information Systems Department, Technical University of Vienna, 1994.

global ca

if ca(i,j)>-1
    %delta=ca(i,j);
elseif i==1 && j==1
    ca(i,j)=norm(P(1,:)-Q(1,:));
elseif i>1 && j==1
    ca(i,j)=max([c(i-1,1,P,Q),norm(P(i,:)-Q(1,:))]);
elseif i==1 && j>1
    ca(i,j)=max([c(1,j-1,P,Q),norm(P(1,:)-Q(j,:))]);
elseif i>1 && j>1
    ca(i,j)=max([min([c(i-1,j,P,Q),c(i-1,j-1,P,Q),c(i,j-1,P,Q)]),norm(P(i,:)-Q(j,:))]);
else
    ca(i,j)=inf;
end

delta=ca(i,j);
end