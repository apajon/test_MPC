%% Plot results
figure(1)
clf
hold on
plot(T:T:phase_duration_cumul(end),z_ref_max(1:end-N),':k','LineWidth',2)
plot(T:T:phase_duration_cumul(end),z_ref_min(1:end-N),':k','LineWidth',2)
% plot(T:T:phase_duration_cumul(end),z_ref,'k')
plot((0:length(x)-1)*T,x,'b')
plot((0:length(z)-1)*T,z,'r')
hold off
