%% foot traj in the air
pstep=[MPC_outputs_storage.xstep MPC_outputs_storage.ystep];
psi=zeros(size(pstep,1)*3,1);
% firstSS=(experiment.phase_type(2)=='r');
%%
pstep_3d=[pstep MPC_outputs_storage.zstep];
pstep_3d_phase_r=[];
pstep_3d_phase_l=[];
for i=1:length(experiment.phase_type)
    if experiment.phase_type(i)=='b'||experiment.phase_type(i)=='start'||experiment.phase_type(i)=='stop'
        if i<length(experiment.phase_type)
            if experiment.phase_type(i+1)=='r'
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
            else
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
            end
        else
            if experiment.phase_type(i-1)=='l'
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(2,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);];
            else
                pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);];
                pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(2,:);];
            end
        end
    pstep_3d(1,:)=[];
    elseif experiment.phase_type(i)=='r'
        pstep_3d_phase_r=[pstep_3d_phase_r;pstep_3d(1,:);pstep_3d(1,:);];
        pstep_3d_phase_l=[pstep_3d_phase_l;...
            [(pstep_3d_phase_l(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_l(end,3),pstep_3d(2,3))+hstep_move];...
            pstep_3d(2,:);];
    elseif experiment.phase_type(i)=='l'
        pstep_3d_phase_l=[pstep_3d_phase_l;pstep_3d(1,:);pstep_3d(1,:);];
        pstep_3d_phase_r=[pstep_3d_phase_r;...
            [(pstep_3d_phase_r(end,1:2)+pstep_3d(2,1:2))./2 max(pstep_3d_phase_r(end,3),pstep_3d(2,3))+hstep_move];...
             pstep_3d(2,:);];
    else
        error('Unkown phase type')
    end
end
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
phase_type_enlarge=ones(length(experiment.phase_type) + length(experiment.phase_type(2:2:end)),1);
phase_type_enlarge(:) = nan;
phase_type_enlarge(3:3:end,:)=char(experiment.phase_type(2:2:end,:));
locations = any(isnan(phase_type_enlarge),2);
phase_type_enlarge(locations) = ['b';char(experiment.phase_type(2:end-1));'b'];
phase_type_enlarge=char(phase_type_enlarge);

phase_duration_enlarge=zeros(length(phase_type_enlarge),1);
phase_duration_enlarge(any(phase_type_enlarge=='r',2))=experiment.phase_duration_r/2;
phase_duration_enlarge(any(phase_type_enlarge=='l',2))=experiment.phase_duration_l/2;
phase_duration_enlarge(any(phase_type_enlarge=='b',2))=experiment.phase_duration_b;
phase_duration_enlarge(1)=experiment.phase_duration_start;
phase_duration_enlarge(end)=experiment.phase_duration_stop;

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
    viapoint_l(k+1,1)=min((viapoint_l(k,1)-viapoint_l(k-3,1))/(experiment.phase_duration_l/2),(-viapoint_l(k,1)+viapoint_l(k+3,1))/(experiment.phase_duration_l/2));
end
for k=4:3:size(viapoint_l,1)-3
    viapoint_l(k+1,2)=min((viapoint_l(k,2)-viapoint_l(k-3,2))/(experiment.phase_duration_l/2),(-viapoint_l(k,2)+viapoint_l(k+3,2))/(experiment.phase_duration_l/2));
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
    viapoint_r(k+1,1)=min((viapoint_r(k,1)-viapoint_r(k-3,1))/(experiment.phase_duration_r/2),(-viapoint_r(k,1)+viapoint_r(k+3,1))/(experiment.phase_duration_r/2));
end
for k=4:3:size(viapoint_r,1)-3
    viapoint_r(k+1,2)=min((viapoint_r(k,2)-viapoint_r(k-3,2))/(experiment.phase_duration_r/2),(-viapoint_r(k,2)+viapoint_r(k+3,2))/(experiment.phase_duration_r/2));
end

%
A_f_r=A_f*viapoint_r;
%%
% dt_time=zeros(length(experiment.phase_type_enlarge),size(A_f_l,1));
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
% ax1 = subplot(3,1,1);
% ylabel('x [m]') % y-axis label
% hold on
% plot(pankle_r(:,1))
% plot(pankle_l(:,1))
% hold off
% 
% ax2 = subplot(3,1,2);
% ylabel('y [m]') % y-axis label
% hold on
% plot(pankle_r(:,2))
% plot(pankle_l(:,2))
% hold off
% 
% ax3 = subplot(3,1,3);
% ylabel('z [m]') % y-axis label
% hold on
% plot(pankle_r(:,3))
% plot(pankle_l(:,3))
% hold off
% linkaxes([ax1,ax2,ax3],'x')
% legend('R foot','Lfoot','Location','southeast')