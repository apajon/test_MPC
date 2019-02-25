%     figure(2)
%     clf
%     title('MPC CoP trajectory')
%     xlabel('t [s]') % x-axis label
%     ylabel('position [m]') % y-axis label
%     axis([0 16 -0.5 4])
%     hold on
%     plot([0;phase_duration_sampling_cumul(1:i)],1*xc+0*xdc-(zc-zzmp_ref(1:i+1))./(zddc+g).*xddc,'b')
%     plot([0;phase_duration_sampling_cumul(1:i)],1*yc+0*ydc-(zc-zzmp_ref(1:i+1))./(zddc+g).*yddc,'g')
%     plot([0;phase_duration_sampling_cumul(1:i)],zzmp_ref(1:i+1),'k')
%     plot(phase_duration_sampling_cumul(i:i+N-1),xf_c+Pu_c*QP_result_all{i}(1:N)-...
%         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
%         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
%         (xf_ddc+Pu_ddc*QP_result_all{i}(1:N)),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),yf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)-...
%         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
%         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
%         (yf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),zzmp_ref(i+1:i+N),'r')
%     legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
%     hold off

    
%     figure(1)
%     clf
%     title('MPC COM trajectory')
%     xlabel('t [s]') % x-axis label
%     ylabel('position [m]') % y-axis label
%     axis([0 16 -0.5 4])
%     hold on
%     plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.xc(1:i+1),'b')
%     plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.yc(1:i+1),'g')
%     plot([0;phase_duration_sampling_cumul(1:i)],MPC_outputs_storage.zc(1:i+1),'k')
%     plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,1)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(1:N),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,2)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r')
%     plot(phase_duration_sampling_cumul(i:i+N-1),COM_state_preview.f_c(:,3)+COM_state_preview.Pu_c*MPC_outputs_storage.QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N),'r')
%     legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
%     hold off
    
% %     figure(2)
% %     clf
% %     title('MPC CoP trajectory')
% %     xlabel('t [s]') % x-axis label
% %     ylabel('position [m]') % y-axis label
% %     axis([0 16 -0.5 4])
% %     hold on
% %     plot([0;phase_duration_sampling_cumul(1:i)],1*xc+0*xdc-(zc-zzmp_ref(1:i+1))./(zddc+g).*xddc,'b')
% %     plot([0;phase_duration_sampling_cumul(1:i)],1*yc+0*ydc-(zc-zzmp_ref(1:i+1))./(zddc+g).*yddc,'g')
% %     plot([0;phase_duration_sampling_cumul(1:i)],zzmp_ref(1:i+1),'k')
% %     plot(phase_duration_sampling_cumul(i:i+N-1),xf_c+Pu_c*QP_result_all{i}(1:N)-...
% %         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
% %         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
% %         (xf_ddc+Pu_ddc*QP_result_all{i}(1:N)),'r')
% %     plot(phase_duration_sampling_cumul(i:i+N-1),yf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)-...
% %         (zf_c+Pu_c*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)-zzmp_ref(i+1:i+N))./...
% %         (zf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)+1:size(A_zmp,2)+N)+g).*...
% %         (yf_ddc+Pu_ddc*QP_result_all{i}(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N)),'r')
% %     plot(phase_duration_sampling_cumul(i:i+N-1),zzmp_ref(i+1:i+N),'r')
% %     legend('x coordinate','y coordinate','z coordinate','preview MPC','Location','northwest')
% %     hold off
    
    if movie_record
        F_COM=getframe(figure(1));
        F_CoP=getframe(figure(2));
        
        writeVideo(v_COM,F_COM);
        writeVideo(v_CoP,F_CoP);
    end