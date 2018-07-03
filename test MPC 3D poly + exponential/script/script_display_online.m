%% Display
    figure(1)
    clf
    title('trajectories along y')
    xlabel('t [s]') % x-axis label
    ylabel('y [m]') % y-axis label
    hold on
    plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),'-*g')
    plot([1:i+1]*T,yc(1:i+1),'-*k')
    plot([(1:N)+i]*T,Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'b') 
%     plot([(1:N)+i]*T,Px_z*[yc(i);ydc(i);yddc(i)]+Pu_z_mean*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'r')

%     plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-(h_com+h_com_max)/g*yddc(1:i+1),'-*r')
%     plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-(h_com+h_com_min)/g*yddc(1:i+1),'-*m')
    plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),'-*r')
    plot([1:i+1]*T,1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),'-*m')
%     if i>=10
%         plot(26:26+6,repmat(-0.075,7,1),'--k')
%         plot(26:26+6,repmat(-0.2050,7,1),'--k')
%     end

%     if i>=18
%         plot(26+8:26+6+8,repmat(QP_result(17)-inttoankle,7,1),'--k')
%         plot(26+8:26+6+8,repmat(QP_result(17)+exttoankle,7,1),'--k')
%     end
%     
%     if i>=25
%         plot(26+16:26+6+16,repmat(QP_result(18)+inttoankle,7,1),'--k')
%         plot(26+16:26+6+16,repmat(QP_result(18)-exttoankle,7,1),'--k')
%     end

    plot([1:i+1]*T,yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),'-*b')

    hold off
    legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

    figure(2)
    clf
    title('trajectories along x')
    xlabel('t [s]') % x-axis label
    ylabel('x [m]') % y-axis label
    hold on
    plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),'-*g')
    plot([1:i+1]*T,xc(1:i+1),'-*k')
    plot(((1:N)+i)*T,Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),'b') 
%     plot((1:N)+i,Px_z*[xc(i);xdc(i);xddc(i)]+Pu_z_mean*QP_result(1:N),'r')

    plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),'-*r')
    plot([1:i+1]*T,1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),'-*m')

%     plot(1:i+N,0.2*(1:i+N)*T)

    plot([1:i+1]*T,xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),'-*b')

    hold off
    legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

    display=true;

    if display
        figure(3)
        clf
        title('trajectories along z')
        xlabel('t [s]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis
        hold on
        plot([1:i+1]*T,zc(1:i+1),'-*k')
        plot(((1:N)+i)*T,Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),'b') 
        hold off
        legend('COM','COM preview','Location','southeast')


        figure(4)
        clf
        title('trajectory of COM on sagittal plane')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        legend('Location','southeast')
        hold on
        plot(xc(1:i+1),zc(1:i+1),'-*k')
        hold off

        figure(5)
        clf
        title('trajectories of COM on lateral plane')
        xlabel('y [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        legend('Location','southeast')
        hold on
        plot(yc(1:i+1),zc(1:i+1),'-*k')
        hold off
    end


    figure(6)
    clf
    title('trajectories along y')
    xlabel('x [m]') % x-axis label
    ylabel('y [m]') % y-axis label
    hold on
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),'-*g')
    plot(xc(1:i+1),yc(1:i+1),'-*k')
    plot(Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),'b') 

    plot(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),'-*r')
    plot(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),'-*m')

    plot(xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),'-*b')

    hold off
    legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

    hold on;
    pstep=[xstep ystep];
    psi=zeros(size(pstep,1)*3,1);
    firstSS=(phase_type(2)=='r');

    XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
    for j=1:length(pstep)
        plot(XY(j,1:5),XY(j,6:10),'-k','LineWidth',2)
    end

    XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
    for j=1:length(pstep)
        plot(XY(j,1:5),XY(j,6:10),':k','LineWidth',2)
    end
    hold off
%     figure(3)
%             clf
%             title('trajectories along z')
%             xlabel('t [s]') % x-axis label
%             ylabel('z [m]') % y-axis label
%             hold on
%             plot([1:i+1]*T,zc(1:i+1),'-*k')
%             plot(((1:N)+i)*T,Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),'b') 
%             hold off
%             legend('COM','COM preview','Location','southeast')
%             
%             figure(4)
%             clf
%             hold on
%             plot(zc(1:i+1)./(zddc(1:i+1)+g))
%             plot(zeta_up_ref,'r')
%             plot(zeta_down_ref,'m')
%             hold off
    figure(6)
    clf
    title('trajectories along y')
    xlabel('x [m]') % x-axis label
    ylabel('y [m]') % y-axis label
    zlabel('z [m]') % z-axis label
    view(3)
    axis([-0.5 4 -2.25 2.25 -inf 1])
    hold on
    plot3(1*xc(1:i+1)+0*xdc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*xddc(1:i+1),...
        1*yc(1:i+1)+0*ydc(1:i+1)-zc(1:i+1)./(zddc(1:i+1)+g).*yddc(1:i+1),...
        [zzmp_ref(1);zzmp_ref(1:i)],...
        '-*g')
    plot3(xc(1:i+1),yc(1:i+1),zc(1:i+1),'-*k')
    plot3(Px_c*[xc(i);xdc(i);xddc(i)]+Pu_c*QP_result(1:N),...
        Px_c*[yc(i);ydc(i);yddc(i)]+Pu_c*QP_result(size(A_zmp,2)/2+1:size(A_zmp,2)/2+N),...
        Px_c*[zc(i);zdc(i);zddc(i)]+Pu_c*QP_result(size(A_zmp,2)+1:size(A_zmp,2)+N),...
        'b') 

    plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_max)/g*xddc(1:i+1),...
        1*yc(1:i+1)+0*ydc(1:i+1)-zeta_up_ref(1)*yddc(1:i+1),...
        [zzmp_ref(1);zzmp_ref(1:i)],...
        '-*r')
    plot3(1*xc(1:i+1)+0*xdc(1:i+1)-(h_com+h_com_min)/g*xddc(1:i+1),...
        1*yc(1:i+1)+0*ydc(1:i+1)-zeta_down_ref(1)*yddc(1:i+1),...
        [zzmp_ref(1);zzmp_ref(1:i)],...
        '-*m')

    plot3(xc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*xdc(1:i+1),...
        yc(1:i+1)+(zc(1:i+1)./(zddc(1:i+1)+g)).^(1/2).*ydc(1:i+1),...
        [zzmp_ref(1);zzmp_ref(1:i)],...
        '-*b')

    hold off
    legend('CoP','COM','COM preview','CoP up','CoP down','Capture point','Location','southeast')

    hold on;
    pstep=[xstep ystep];
    psi=zeros(size(pstep,1)*3,1);
    firstSS=(phase_type(2)=='r');

    XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
    for j=1:length(pstep)
        plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
    end

    XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
    for j=1:length(pstep)
        plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
    end

    XY=drawing_rectangle_rotate(pstep,psi,backtoankle+sole_margin,fronttoankle+sole_margin,exttoankle+sole_margin,inttoankle+sole_margin,firstSS);
    for j=1:length(pstep)
        fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
    end
    hold off