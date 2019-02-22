% pstep_3d=[pstep zstep];
% 
% save results.mat xc_discret yc_discret zc_discret ...
%             xdc_discret ydc_discret zdc_discret ...
%             xddc_discret yddc_discret zddc_discret ...
%             xc yc zc ...
%             xdc ydc zdc ...
%             xddc yddc zddc ...
%             xdddc_storage ydddc_storage zdddc_storage ...
%             pstep_3d ...
%             dt_type_phase_ ...
%             phase_type_sampling
            
            
%%
clear all
close all

load test_foot_traj_kinematics/results.mat
%%
N=find(phase_type_sampling=="b",1)+1;
M=find(phase_type_sampling=="b",2)-1;
M(1)=[];

com=[xc(N:M) yc(N:M) zc(N:M)];
dc=[xdc(N:M) ydc(N:M) zdc(N:M)];
ddc=[xddc(N:M) yddc(N:M) zddc(N:M)];
dddc=[xdddc_storage(N:M) ydddc_storage(N:M) zdddc_storage(N:M)];

step1=pstep_3d(2,:);
step2=pstep_3d(4,:);

step1(:,3)=0.093;
step2(:,3)=0.093;

x_ankle=step1(1);
z_ankle=step1(3);

R1 = 0.358;
R2 = 0.341;
R3 = 0.0875;

%init
q1=[-1.18446];
q2=[-1.29001];

q1_=[-0.813392];
q2_=[-0.869249];

dq1=0;
dq2=0;

ddq1=0;
ddq2=0;

dddq1=[];
dddq2=[];

w1=1;10^-6; %Jerk
w2=1; %close to step final 

%%
figure(1)
clf
axis equal
hold on
plot(com(1,1),com(1,3),'xb')
plot(step1(1,1),step1(1,3),'xb')
toto=com(1,[1 3])-[0 R3];
plot(toto(1,1),toto(1,2),'ob')
toto1=toto+R1*[cos(q1) sin(q1)];
plot(toto1(1,1),toto1(1,2),'ob')
toto2=toto1+R2*[cos(q1+q2) sin(q1+q2)];
plot(toto2(1,1),toto2(1,2),'ob')
plot([toto(1,1) toto1(1,1)],[toto(1,2) toto1(1,2)],'b')
plot([toto1(1,1) toto2(1,1)],[toto1(1,2) toto2(1,2)],'b')

plot(com(end,1),com(end,3),'xb')
plot(step2(1,1),step2(1,3),'xb')
toto=com(end,[1 3])-[0 R3];
plot(toto(1,1),toto(1,2),'ob')
toto1=toto+R1*[cos(q1_) sin(q1_)];
plot(toto1(1,1),toto1(1,2),'ob')
toto2=toto1+R2*[cos(q1_+q2_) sin(q1_+q2_)];
plot(toto2(1,1),toto2(1,2),'ob')
plot([toto(1,1) toto1(1,1)],[toto(1,2) toto1(1,2)],'b')
plot([toto1(1,1) toto2(1,1)],[toto1(1,2) toto2(1,2)],'b')

plot([step1(1,1)-0.2 step2(1,1)+0.2],[0 0],'k','LineWidth',4)
hold off

%%
for k=1:1%size(c,1)
    Nsample=size(com,1)-k+1;
    %%
    J=jacobianFoot(q1(k),q2(k),R1,R2);
    %%
    [Px_q,Pu_q,Px_dq,Pu_dq,Px_ddq,Pu_ddq]=function_compute_com_linear_comJerk(ones(Nsample,1)*0.1,Nsample);
    %% generation of q1 and q2
    Pu_q1=[Pu_q Pu_q*0];
    Pu_q2=[Pu_q*0 Pu_q];
    
    f_q1=Px_q*[q1(k);dq1(k);ddq1(k)];
    f_q2=Px_q*[q2(k);dq2(k);ddq2(k)];
    
    Pu_dq1=[Pu_dq Pu_dq*0];
    Pu_dq2=[Pu_dq*0 Pu_dq];
    
    f_dq1=Px_dq*[q1(k);dq1(k);ddq1(k)];
    f_dq2=Px_dq*[q2(k);dq2(k);ddq2(k)];
    
    Pu_ddq1=[Pu_ddq Pu_ddq*0];
    Pu_ddq2=[Pu_ddq*0 Pu_ddq];
    
    f_ddq1=Px_ddq*[q1(k);dq1(k);ddq1(k)];
    f_ddq2=Px_ddq*[q2(k);dq2(k);ddq2(k)];
    %% foot traj
    % diff movement in COM frame
    xPu_J_ankle=J(1,1)*(Pu_q1(2:end,:)-Pu_q1(1:end-1,:) )...
        +J(1,2)*(Pu_q2(2:end,:)-Pu_q2(1:end-1,:));
    zPu_J_ankle=J(2,1)*(Pu_q1(2:end,:)-Pu_q1(1:end-1,:))...
        +J(2,2)*(Pu_q2(2:end,:)-Pu_q2(1:end-1,:));
    
    xf_J_ankle=J(1,1)*(f_q1(2:end)-f_q1(1:end-1))...
        +J(1,2)*(f_q2(2:end)-f_q2(1:end-1))...
        +dc(2:end,1);
    zf_J_ankle=J(2,1)*(f_q1(2:end)-f_q1(1:end-1))...
        +J(2,2)*(f_q2(2:end)-f_q2(1:end-1))...
        +dc(2:end,1);
    
    %
    xPu_ankle=cumsum(xPu_J_ankle,1);
    zPu_ankle=cumsum(zPu_J_ankle,1);
    
    xf_ankle=x_ankle(k)+cumsum(com(2:end,1)-com(1:end-1,1),1)+cumsum(xf_J_ankle,1);
    zf_ankle=z_ankle(k)+cumsum(com(2:end,3)-com(1:end-1,3),1)+cumsum(zf_J_ankle,1);
    
    
    %%
    H_jerk=eye(Nsample*2);
    f_jerk=zeros(Nsample*2,1);
    
% %     pu_q_=Pu_q;
% %     pu_q_(end,:)=0;
% %     Hposition=Pu_q_.'*Pu_q_;
%     xPu_c_=xPu_J_ankle;
%     xPu_c_(end,:)=0;
%     xHposition=xPu_c_.'*xPu_c_;
%     zPu_c_=zPu_J_ankle;
%     zPu_c_(end,:)=0;
%     zHposition=zPu_c_.'*zPu_c_;
%     
%     H=w1*Hjerk+w2*xHposition+w2*zHposition;
%     
%     xf_c_=xf_J_ankle;
%     xf_c_(end,:)=0;
%     zf_c_=zf_J_ankle;
%     zf_c_(end,:)=0;
%     
%     xf=xPu_c_.'*(xf_c_-[ones(size(xf_c_,1)-1,1)*step2(1);0]);
%     zf=zPu_c_.'*(zf_c_-[ones(size(zf_c_,1)-1,1)*step2(3);0]);
%     
%     f=w2*xf+w2*zf;

    H=w1*H_jerk;
    f=w1*f_jerk;

    %%
    Aeq_ankle_final=[xPu_ankle(end,:);zPu_ankle(end,:)];
    beq_ankle_final=[step2(1)-xf_ankle(end,:);step2(3)-zf_ankle(end,:)];
    
    Aeq=Aeq_ankle_final;
    beq=beq_ankle_final;
    
    %% constraints
    A=[];b=[];
%     Aeq=[];beq=[];
    lb=[];ub=[];
    x0=[];

    %% Options
%     options=optimoptions('quadprog','Display','iter','MaxIterations',1000);
    options=optimoptions('quadprog','Display','final');
%     options=optimoptions('quadprog','Display','off');

    %% Optimization QP
    [QP_result_all{k},tata,converge_sampling(k)]=quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    
%     %%
%     q1_=Pu_q1*QP_result_all{k}+f_q1;
%     q2_=qPu_q2*QP_result_all{k}+f_q2;
%     
%     figure(2)
%     clf
%     axis equal
%     hold on
%     plot(q1(1),step1(3),'or')
%     plot(step2(1),step2(3),'or')
%     plot(q1_,'-*')
%     plot(q2_,'-*')
%     hold off
    %%
%     xankle=xPu_ankle*QP_result_all{k}+xf_ankle;
%     zankle=zPu_ankle*QP_result_all{k}+zf_ankle;
%     
%     q1_=Pu_q1*QP_result_all{k}+f_q1;
%     q2_=Pu_q2*QP_result_all{k}+f_q2;
%     
%     
%     
%     figure(3)
%     clf
%     axis equal
%     hold on
%     plot(step1(1),step1(3),'or')
%     plot(step2(1),step2(3),'or')
%     plot(com(:,1),com(:,3),'xr')
%     plot(xankle,zankle,'-*')
%     
%     toto=com(1:end,[1 3])-[0 R3];
%     plot(toto(:,1),toto(:,2),'or')
%     toto1=toto+R1*[cos(q1_) sin(q1_)];
%     plot(toto1(:,1),toto1(:,2),'ob')
%     toto2=toto1+R2*[cos(q1_+q2_) sin(q1_+q2_)];
%     plot(toto2(:,1),toto2(:,2),'ob')
%     plot([toto(:,1) toto1(:,1)],[toto(:,2) toto1(:,2)],'b')
%     plot([toto1(:,1) toto2(:,1)],[toto1(:,2) toto2(:,2)],'b')
%     hold off

end


