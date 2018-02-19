clear all

%% Constant
g=9.81; %m.s-1
h_com=0.8; %m
Q=1; R=Q*10^-6; %R/Q=10^-6
T=5*10^-3; %s
N=300;

T=5*10^-2;
N=30;

%% Initial Robot State
x_0=[0;0;0];

%% Phase duration
phase_type=['b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b'];
phase_duration=zeros(length(phase_type),1);
phase_duration(any(phase_type=='r',2))=0.7;
phase_duration(any(phase_type=='l',2))=0.7; %
phase_duration(any(phase_type=='b',2))=0.1;
phase_duration(1)=2.4;
phase_duration(end)=10;

phase_duration_cumul=zeros(length(phase_duration),1);
for i=1:length(phase_duration)
    phase_duration_cumul(i,1)=sum(phase_duration(1:i,1));
end

%% ZMP admissible
z_max_b=0.175-0.04;
z_min_b=-0.175+0.04;

z_max_r=-0.0375-0.04;
z_min_r=-0.175+0.04;

z_max_l=0.175-0.04;
z_min_l=0.0375+0.04;

%% ZMP ref discretization
z_ref=zeros(round(max(phase_duration_cumul)/T),1);
z_ref_max=zeros(round(max(phase_duration_cumul)/T),1);
z_ref_min=zeros(round(max(phase_duration_cumul)/T),1);

for i=1:length(z_ref)
   [scrap,phase_number]=max(phase_duration_cumul>i*T);
   
   if phase_type(phase_number)=='r'
       z_ref_max(i)=z_max_r;
       z_ref_min(i)=z_min_r;
   elseif phase_type(phase_number)=='l'
       z_ref_max(i)=z_max_l;
       z_ref_min(i)=z_min_l;

   else %phase_type(phase_number)=='b'
       z_ref_max(i)=z_max_b;
       z_ref_min(i)=z_min_b;

   end
end
z_ref=(z_ref_max+z_ref_min)./2;
z_ref=[z_ref;z_ref(end)*ones(N,1)];

z_ref_max=[z_ref_max;z_ref_max(end)*ones(N,1)];
z_ref_min=[z_ref_min;z_ref_min(end)*ones(N,1)];

%% ZMP Linear form
%Z_k+1=P_x * x0_k+ P_u * dddX_k
%x0_k=[x(kT);dx(kT);ddx(kT)]
%dddX_k=[dddx_k; ... ;dddx_k+N-1]

P_x=zeros(N,3);
for i=1:N
    P_x(i,1:3)=[1 i*T i^2*T^2/2-h_com/g];
end

P_u=zeros(N,N);
for i=1:N
    for j=1:i
       P_u(i,j)=T^3/6+(i-j)*T^3/2+T*(-h_com/g+1/2*(i-j)^2*T^2); 
    end
end

%% Optimization problem QP
% Init storage
x=zeros(N+1,1);
dx=zeros(N+1,1);
ddx=zeros(N+1,1);
dddx_storage=[];

x(1)=x_0(1);
dx(1)=x_0(2);
ddx(1)=x_0(3);

% Jerk
dddx2_H=eye(N);
dddx2_f=zeros(1,N);


tic
for i=1:round(max(phase_duration_cumul)/T)
    % ZMP
    z_H=P_u;
    z_f=P_x*[x(i);dx(i);ddx(i)];
    
    z2_H=P_u.'*P_u;
    z2_f=z_f.'*P_u;
    
    % Cost
    H=dddx2_H;
    f=dddx2_f;
    
    % Constraints inequalities
    z_max=z_ref_max(1+(i-1):N+(i-1));
    z_min=z_ref_min(1+(i-1):N+(i-1));
    
    A=[z_H;...
        -z_H];
    b=[z_max-z_f;...
        -z_min+z_f];
    
    %constraint
    Aeq=[];beq=[];lb=[];ub=[];x0=[];

    % Options
%     options=optimoptions('quadprog','Display','iter');
    options=optimoptions('quadprog','Display','off');

    % Optimization QP
    dddx=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    
    dddx_storage=[dddx_storage;dddx(1)];
    
    % Results COM
    x(i+1)=[1 T T^2/2]*[x(i);dx(i);ddx(i)]+T^3/6*dddx(1);
    dx(i+1)=[0 1 T]*[x(i);dx(i);ddx(i)]+T^2/2*dddx(1);
    ddx(i+1)=[0 0 1]*[x(i);dx(i);ddx(i)]+T*dddx(1);
    
end
toc

% Results ZMP
z=1*x+0*dx-h_com/g*ddx;
%% Plot
figure(1)
clf
hold on
plot(T:T:phase_duration_cumul(end),z_ref_max(1:end-N),':k','LineWidth',2)
plot(T:T:phase_duration_cumul(end),z_ref_min(1:end-N),':k','LineWidth',2)
% plot(T:T:phase_duration_cumul(end),z_ref,'k')
plot((0:length(x)-1)*T,x,'b')
plot((0:length(z)-1)*T,z,'r')
hold off
