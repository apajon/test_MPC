clear all

%% Constant
% Mechanics
g=9.81; %m.s-1
h_com=0.8; %m

% Sampling time
T=5*10^-3;
N=300;

T=5*10^-2;
N=30;

T=0.1;
N=16;

%% Initial Robot State
xcom_0=[0;0;0];
ycom_0=[0;0;0];

%% Phase duration
phase_type=['b';'r';'b';'l';'b';'r';'b';'l';'b';'r';'b';'l';'b'];
phase_duration=zeros(length(phase_type),1);
phase_duration(any(phase_type=='r',2))=0.7;
phase_duration(any(phase_type=='l',2))=0.7; %
phase_duration(any(phase_type=='b',2))=0.1;
phase_duration(1)=2.4;
phase_duration(end)=2.4;

phase_duration_cumul=zeros(length(phase_duration),1);
for i=1:length(phase_duration)
    phase_duration_cumul(i,1)=sum(phase_duration(1:i,1));
end

%% Foot limits
backtoankle=0.1; %from back to ankle of foot
fronttoankle=0.13; %from  front to ankle of foot
exttoankle=0.075; %from exterior to ankle of foot
inttoankle=0.055; %from interior to ankle of foot

%% Foot step placement limits
xankmax=0.4;%stepping forward max
xankmin=-0.4;%stepping forward min (if negative, it means stepping backward max)
yankmin=inttoankle+0.03;%width min between ankles
yankmax=inttoankle+0.4;%width max between ankles

%% COM vel ref
xvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);
yvcom_ref=zeros(round(max(phase_duration_cumul)/T),1);

xvcom_ref=xvcom_ref+0.2; %m.s^-1

xvcom_ref=[xvcom_ref;xvcom_ref(end)*ones(N,1)];

%% ZMP Linear form
%Z_k+1=P_x * x0_k+ P_u * dddX_k
%x0_k=[x(kT);dx(kT);ddx(kT)]
%dddX_k=[dddx_k; ... ;dddx_k+N-1]

Pz_x=zeros(N,3);
for i=1:N
    Pz_x(i,1:3)=[1 i*T i^2*T^2/2-h_com/g];
end

Pz_u=zeros(N,N);
for i=1:N
    for j=1:i
       Pz_u(i,j)=T^3/6+(i-j)*T^3/2+T*(-h_com/g+1/2*(i-j)^2*T^2); 
    end
end

%% COM Linear form
% COM position
Pc_x=zeros(N,3);
for i=1:N
    Pc_x(i,1:3)=[1 i*T i^2*T^2/2];
end

Pc_u=zeros(N,N);
for i=1:N
    for j=1:i
       Pc_u(i,j)=T^3/6+(i-j)*T^3/2+(i-j)^2*T^3/2; 
    end
end

% COM velocity
Pdc_x=zeros(N,3);
for i=1:N
    Pdc_x(i,1:3)=[0 1 i*T];
end

Pdc_u=zeros(N,N);
for i=1:N
    for j=1:i
       Pdc_u(i,j)=T^2/2+(i-j)*T; 
    end
end

% COM acceleration
Pddc_x=zeros(N,3);
for i=1:N
    Pddc_x(i,1:3)=[0 0 1];
end

Pddc_u=zeros(N,N);
for i=1:N
    for j=1:i
       Pddc_u(i,j)=T; 
    end
end
%% Optimization problem QP
% Init storage
xc=zeros(N+1,1);
xdc=zeros(N+1,1);
xddc=zeros(N+1,1);
xdddc_storage=[];

xc(1)=xcom_0(1);
xdc(1)=xcom_0(2);
xddc(1)=xcom_0(3);

yc=zeros(N+1,1);
ydc=zeros(N+1,1);
yddc=zeros(N+1,1);
ydddc_storage=[];

yc(1)=ycom_0(1);
ydc(1)=ycom_0(2);
yddc(1)=ycom_0(3);

% Jerk cost
% min sum j^2
xH_dddc2=eye(N);
xf_dddc2=zeros(1,N);

yH_dddc2=eye(N);
yf_dddc2=zeros(1,N);

% Sampling update
tic
for i=1:round(max(phase_duration_cumul)/T)
    % COM velocity
    % min sum (dc-dc_ref)^2
    xH_dc=Pdc_u.'*Pdc_u;
    xf_dc=Pdc_x*[xc(i);xdc(i);xddc(i)]-xvcom_ref(1-(i-1):N-(i-1));
    
    yH_dc=Pdc_u.'*Pdc_u;
    yf_dc=Pdc_x*[yc(i);ydc(i);yddc(i)]-yvcom_ref(1-(i-1):N-(i-1));
    
    % Cost
    xH=xH_dc...
        +xH_dddc2;
    xf=xf_dc...
        +xf_dddc2;
    
    yH=yH_dc...
        +yH_dddc2;
    yf=yf_dc...
        +yf_dddc2;
    
    % Constraints inequalities
    A=[];
    b=[];
    
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
