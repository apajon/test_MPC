%% foot traj in the air
% hstep_move=0.05;
% % hstep_move=0.2;
% hstep_move_mid=0.07;

pstep=[xstep ystep];
pstep_3d=[pstep zstep];
psi=zeros(size(pstep,1)*3,1);
firstSS=(phase_type(2)=='r');
%%
xpstep_viapoint=zeros(4,size(pstep_3d,1)-2);
ypstep_viapoint=zeros(4,size(pstep_3d,1)-2);
zpstep_viapoint=zeros(4,size(pstep_3d,1)-2);
for k=1:size(pstep_3d,1)-2
    xpstep_viapoint(1,k)=pstep_3d(k,1);
    ypstep_viapoint(1,k)=pstep_3d(k,2);
    zpstep_viapoint(1,k)=pstep_3d(k,3);
    
    if pstep_3d(k,3)>pstep_3d(k+1,3)
        xpstep_viapoint(2,k)=pstep_3d(k+1,1);
        ypstep_viapoint(2,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*1/4;
        zpstep_viapoint(2,k)=pstep_3d(k,3)+hstep_move;
    elseif pstep_3d(k,3)<pstep_3d(k+1,3)
        xpstep_viapoint(2,k)=pstep_3d(k,1);
        ypstep_viapoint(2,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*1/4;
        zpstep_viapoint(2,k)=pstep_3d(k+1,3)+hstep_move;
    elseif pstep_3d(k,3)==pstep_3d(k+1,3)
        xpstep_viapoint(2,k)=pstep_3d(k,1)+(pstep_3d(k+2,1)-pstep_3d(k,1))*1/4;
        ypstep_viapoint(2,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*1/4;
        zpstep_viapoint(2,k)=pstep_3d(k+1,3)+hstep_move;
    else
        msg='Bad Step height \n';
        errormsg=[msg];
        error(errormsg,[])
    end
    
    if pstep_3d(k+1,3)>pstep_3d(k+2,3)
        xpstep_viapoint(3,k)=pstep_3d(k+2,1);
        ypstep_viapoint(3,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*3/4;
        zpstep_viapoint(3,k)=pstep_3d(k+1,3)+hstep_move;
    elseif pstep_3d(k+1,3)<pstep_3d(k+2,3)
        xpstep_viapoint(3,k)=pstep_3d(k+1,1);
        ypstep_viapoint(3,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*3/4;
        zpstep_viapoint(3,k)=pstep_3d(k+2,3)+hstep_move;
    elseif pstep_3d(k+1,3)==pstep_3d(k+2,3)
        xpstep_viapoint(3,k)=pstep_3d(k,1)+(pstep_3d(k+2,1)-pstep_3d(k,1))*3/4;
        ypstep_viapoint(3,k)=pstep_3d(k,2)+(pstep_3d(k+2,2)-pstep_3d(k,2))*3/4;
        zpstep_viapoint(3,k)=pstep_3d(k+1,3)+hstep_move;
    else
        msg='Bad Step height \n';
        errormsg=[msg];
        error(errormsg,[])
    end
    
        xpstep_viapoint(4,k)=pstep_3d(k+2,1);
        ypstep_viapoint(4,k)=pstep_3d(k+2,2);
        zpstep_viapoint(4,k)=pstep_3d(k+2,3);
end
%%
pstep_3d_phase_r=[];
pstep_3d_phase_l=[];
for k=2:2:length(phase_type)
    if k==24
        k
    end
    switch phase_type(k)
        case 'r'
            if k/2+1<size(xpstep_viapoint,2)
                pstep_3d_phase_r=[pstep_3d_phase_r;repmat([xpstep_viapoint(1,k/2+1) ypstep_viapoint(1,k/2+1) zpstep_viapoint(1,k/2+1)],4,1);];
            else
                pstep_3d_phase_r=[pstep_3d_phase_r;repmat(pstep_3d_phase_r(end,:),4,1);];
            end
            pstep_3d_phase_l=[pstep_3d_phase_l;[xpstep_viapoint(:,k/2) ypstep_viapoint(:,k/2) zpstep_viapoint(:,k/2)];];
        case 'l'
            if k/2+1<size(xpstep_viapoint,2)               
                pstep_3d_phase_l=[pstep_3d_phase_l;repmat([xpstep_viapoint(1,k/2+1) ypstep_viapoint(1,k/2+1) zpstep_viapoint(1,k/2+1)],4,1);];
            else
                pstep_3d_phase_l=[pstep_3d_phase_l;repmat(pstep_3d_phase_l(end,:),4,1);];
            end
            pstep_3d_phase_r=[pstep_3d_phase_r;[xpstep_viapoint(:,k/2) ypstep_viapoint(:,k/2) zpstep_viapoint(:,k/2)];];
        otherwise
            msg='Bad Sphase_type \n';
            errormsg=[msg];
            error(errormsg,[])
    end
end
pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d_phase_l(end,:)];
pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d_phase_r(end,:)];

% for i=1:length(phase_type)
%     if phase_type(i)=='b'||phase_type(i)=='start'||phase_type(i)=='stop'
%         if i<length(phase_type)
%             if phase_type(i+1)=='r'
%                 pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
%                 pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
%             else
%                 pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
%                 pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
%             end
%         else
%             if phase_type(i-1)=='l'
%                 pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
%                 pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
%             else
%                 pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
%                 pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
%             end
%         end
%         pstep_3d(1,:)=[];
%     elseif phase_type(i)=='r'
%         pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);pstep_3d(1,:);];
%         pstep_3d_phase_l=[pstep_3d_phase_l;...
%             [(pstep_3d_phase_l(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_l(end,3),pstep_3d(2,3))+hstep_move];...
%             pstep_3d(2,:);];
%     elseif phase_type(i)=='l'
%         pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);pstep_3d(1,:);];
%         pstep_3d_phase_r=[pstep_3d_phase_r;...
%             [(pstep_3d_phase_r(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_r(end,3),pstep_3d(2,3))+hstep_move];...
%              pstep_3d(2,:);];
%     else
%         error('Unkown phase type')
%     end
% end
%%
% figure(8)
% clf
% title('trajectories foot in air')
% 
% ax1 = subplot(3,1,1);
% ylabel('x [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,1))
% plot(pstep_3d_phase_l(:,1))
% hold off
% 
% ax2 = subplot(3,1,2);
% ylabel('y [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,2))
% plot(pstep_3d_phase_l(:,2))
% hold off
% 
% ax3 = subplot(3,1,3);
% ylabel('z [m]') % y-axis label
% hold on
% plot(pstep_3d_phase_r(:,3))
% plot(pstep_3d_phase_l(:,3))
% hold off
% linkaxes([ax1,ax2,ax3],'x')
% legend('R foot','Lfoot','Location','southeast')

%%
phase_type_enlarge=ones(length(phase_type) + length(phase_type(2:2:end))*2,1);
phase_type_enlarge(:) = nan;
phase_type_enlarge(3:4:end,:)=char(phase_type(2:2:end,:));
phase_type_enlarge(4:4:end,:)=char(phase_type(2:2:end,:));
locations = any(isnan(phase_type_enlarge),2);
phase_type_enlarge(locations) = ['b';char(phase_type(2:end-1));'b'];
phase_type_enlarge=char(phase_type_enlarge);

phase_duration_enlarge=zeros(length(phase_type_enlarge),1);
phase_duration_enlarge(any(phase_type_enlarge=='b',2))=phase_duration_b;

locations = find(phase_type_enlarge=='r');
phase_duration_enlarge(locations(1:3:end))=phase_duration_r/4;
phase_duration_enlarge(locations(2:3:end))=phase_duration_r/2;
phase_duration_enlarge(locations(3:3:end))=phase_duration_r/4;

locations = find(phase_type_enlarge=='l');
phase_duration_enlarge(locations(1:3:end))=phase_duration_r/4;
phase_duration_enlarge(locations(2:3:end))=phase_duration_r/2;
phase_duration_enlarge(locations(3:3:end))=phase_duration_r/4;

phase_duration_enlarge(1)=phase_duration_start;
phase_duration_enlarge(end)=phase_duration_stop;

phase_duration_cumul_enlarge=zeros(length(phase_duration_enlarge),1);
for i=1:length(phase_duration_enlarge)
    phase_duration_cumul_enlarge(i,1)=sum(phase_duration_enlarge(1:i,1));
end

[A_f,B_f]=compute_coeff_poly6_interpolation([0;phase_duration_cumul_enlarge],[],[],size(phase_type_enlarge,1),(size(phase_type_enlarge,1)+1)*3);

%%
%   1. Define new vector Y based on existing vector x,
viapoint_l = ones(length(pstep_3d_phase_l)+1 + length(pstep_3d_phase_l)*2+2,3);
viapoint_l(:) = nan;
%2. Insert New.Values into respective cells of Y, for example
viapoint_l(2:3:end,:) = 0;
viapoint_l(3:3:end,:) = 0;
% 3. after all new values inserted, find locations of NaN in Y;
locations = find(isnan(viapoint_l));
% 4. Replace the found locations of Y with existing x values;
viapoint_l(locations) = [pstep_3d_phase_l(1,:);pstep_3d_phase_l;];

for k=4:3:size(viapoint_l,1)-3
    viapoint_l(k+1,1)=min((viapoint_l(k,1)-viapoint_l(k-3,1))/(phase_duration_l/2),(-viapoint_l(k,1)+viapoint_l(k+3,1))/(phase_duration_l/2));
end
for k=4:3:size(viapoint_l,1)-3
    viapoint_l(k+1,2)=min((viapoint_l(k,2)-viapoint_l(k-3,2))/(phase_duration_l/2),(-viapoint_l(k,2)+viapoint_l(k+3,2))/(phase_duration_l/2));
end

%
A_f_l=A_f*viapoint_l;

%   1. Define new vector Y based on existing vector x,
viapoint_r = ones(length(pstep_3d_phase_r)+1 + length(pstep_3d_phase_r)*2+2,3);
viapoint_r(:) = nan;
%2. Insert New.Values into respective cells of Y, for example
viapoint_r(2:3:end,:) = 0;
viapoint_r(3:3:end,:) = 0;
% 3. after all new values inserted, find locations of NaN in Y;
locations = find(isnan(viapoint_r));
% 4. Replace the found locations of Y with existing x values;
viapoint_r(locations) = [pstep_3d_phase_r(1,:);pstep_3d_phase_r;];

for k=4:3:size(viapoint_r,1)-3
    viapoint_r(k+1,1)=min((viapoint_r(k,1)-viapoint_r(k-3,1))/(phase_duration_r/2),(-viapoint_r(k,1)+viapoint_r(k+3,1))/(phase_duration_r/2));
end
for k=4:3:size(viapoint_r,1)-3
    viapoint_r(k+1,2)=min((viapoint_r(k,2)-viapoint_r(k-3,2))/(phase_duration_r/2),(-viapoint_r(k,2)+viapoint_r(k+3,2))/(phase_duration_r/2));
end

%
A_f_r=A_f*viapoint_r;
%%
% dt_time=zeros(length(phase_type_enlarge),size(A_f_l,1));
% for i=1:size(dt_time,1)
%     dt_time(i,(i-1)*6+1)=1;
% end
% 
% toto=dt_time*A_f_l;

frequency=200;
discretization=phase_duration_enlarge*200;

phase_type_sampling_enlarge=[];
for i=1:length(phase_duration_enlarge)
    phase_type_sampling_enlarge=[phase_type_sampling_enlarge;repmat(phase_type_enlarge(i),discretization(i),1)];
end

[mdt] = compute_coeff_dt_matrix(discretization,frequency,length(phase_type_enlarge),length(phase_type_sampling_enlarge)+1);
[mddt] = compute_coeff_ddt_matrix(discretization,frequency);
[mdddt] = compute_coeff_dddt_matrix(discretization,frequency);

pankle_l=mdt*A_f_l;
pankle_r=mdt*A_f_r;

vankle_l=mddt*A_f_l;
vankle_r=mddt*A_f_r;

aankle_l=mdddt*A_f_l;
aankle_r=mdddt*A_f_r;

% %%
% figure(9)
% clf
% title('trajectories foot in air')
% 
% ax1 = subplot(6,1,1);
% ylabel('x [m]') % y-axis label
% hold on
% plot(pankle_r(:,1))
% plot(pankle_l(:,1))
% hold off
% 
% ax2 = subplot(6,1,2);
% ylabel('y [m]') % y-axis label
% hold on
% plot(pankle_r(:,2))
% plot(pankle_l(:,2))
% hold off
% 
% ax3 = subplot(6,1,3);
% ylabel('z [m]') % y-axis label
% hold on
% plot(pankle_r(:,3))
% plot(pankle_l(:,3))
% hold off
% linkaxes([ax1,ax2,ax3],'x')
% legend('R foot','Lfoot','Location','southeast')
% 
% subplot(6,1,4:6);
% xlabel('x [m]') % x-axis label
% ylabel('y [m]') % y-axis label
% zlabel('z [m]') % z-axis label
% view(3)
% hold on
% plot3(pankle_r(:,1),pankle_r(:,2),pankle_r(:,3))
% plot3(pankle_l(:,1),pankle_l(:,2),pankle_l(:,3))
% hold off