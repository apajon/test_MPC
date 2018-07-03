no_begin=50;
no_end=50;
% %% Plot results
% figure(7)
% clf
% 
% ax1 = subplot(2,1,1);
% title('trajectory of COM 3d')
% xlabel('x [m]') % x-axis label
% ylabel('y [m]') % y-axis label
% zlabel('z [m]') % z-axis label
% view(3)
% hold on
% plot3(xc,yc,zc,'-*k')
% hold off
% legend('COM','Location','southeast')
% 
% ax2 = subplot(2,1,2);
% title('trajectories 3d')
% xlabel('x [m]') % x-axis label
% ylabel('y [m]') % y-axis label
% zlabel('z [m]') % z-axis label
% view(3)
% hold on
% plot3(xz,yz,zz,'-*g')
% 
% plot3(xz_up,yz_up,zz_up,'-*r')
% plot3(xz_down,yz_down,zz_down,'-*m')
% 
% plot3(xcapture,ycapture,zcapture,'-*b')
% hold off
% hold on;
% pstep=[xstep ystep];
% psi=zeros(size(pstep,1)*3,1);
% firstSS=(phase_type(2)=='r');
% 
% XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
% for j=1:length(pstep)
%     plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
% end
% 
% XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
% for j=1:length(pstep)
%     plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
% end
% 
% XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
% for j=1:length(pstep)
%     fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
% end
% hold off
% hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'}); 
% rotate3d on
% 
% legend('CoP','CoP up','CoP down','Capture point','Location','southeast')

%% Plot results 3D
figure(8)
clf

title('trajectories 3D')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
axis equal
view(3)
hold on
plot3(xc,yc,zc-h_com*0+0.0,'-*k')
legend('COM','Location','southeast')

plot3(xz,yz,zz,'-*g')

plot3(xz_up,yz_up,zz_up,'-*r')
plot3(xz_down,yz_down,zz_down,'-*m')

plot3(xcapture,ycapture,zcapture,'-*b')

plot3(xdcm,ydcm,zdcm-h_com*0+0.0,'-*b')
hold off
hold on;
XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
for j=1:length(pstep)
    plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
for j=1:length(pstep)
    plot3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),':k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
for j=1:length(pstep)
    fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
end
hold off
legend('COM','CoP','CoP up','CoP down','Capture point','Location','southeast')

%% Plot results top view
figure(13)
clf

title('trajectories 3D')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
axis equal
% axis image
hold on
plot(xc(1:length(xvcom_ref)-no_end),yc(1:length(xvcom_ref)-no_end),'-*k')
legend('COM','Location','southeast')

plot(xz(1:length(xvcom_ref)-no_end),yz(1:length(xvcom_ref)-no_end),'-*g')

plot(xz_up(1:length(xvcom_ref)-no_end),yz_up(1:length(xvcom_ref)-no_end),'-*r')
plot(xz_down(1:length(xvcom_ref)-no_end),yz_down(1:length(xvcom_ref)-no_end),'-*m')

% plot(xcapture,ycapture,'-*b')

% plot(xdcm,ydcm,'-*b')
hold off

hold on;
XY=drawing_rectangle_rotate(pstep,psi,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS);
for j=1:length(pstep)
    plot(XY(j,1:5),XY(j,6:10),'-k','LineWidth',2)
end

XY=drawing_rectangle_rotate(pstep,psi,backtoankle-sole_margin,fronttoankle-sole_margin,exttoankle-sole_margin,inttoankle-sole_margin,firstSS);
for j=1:length(pstep)
    plot(XY(j,1:5),XY(j,6:10),':k','LineWidth',2)
end

        
% plot(p_sommet{1}(1)+xstep(12:13),p_sommet{1}(2),'*b')
% for j=2:number_level
%     plot(p_sommet{j}(1:3:end)+xstep(12:13),p_sommet{j}(2:3:end),'-*b')
% end
% 
% for j=1:number_level-1
%     for k=1:size(edge{j},1)
%          plot(edge{j}(k,1:3:end)+xstep(12:13),edge{j}(k,2:3:end),'-r')
%     end
% end

% XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
% for j=1:length(pstep)
%     fill3(XY(j,1:5),XY(j,6:10),[132,200,225]/255,'LineStyle','none')
% end

% plot(pstep(:,1),pstep(:,2),'o');
% plot(pstep(:,1)+xtranslate_step,pstep(:,2),'*');
hold off
legend('COM','CoP','CoP up','CoP down','Location','southeast')

%% Plot results Zeta
figure(10)
clf
title('zeta')
xlabel('t [s]') % x-axis label
ylabel('zeta [s^2]') % y-axis label
hold on
plot([1:size(zeta_up_ref,1)]*T,zeta_up_ref,'-*')
plot([1:size(zeta_up_ref,1)]*T,zeta_down_ref,'-*')
plot([0:size(zeta,1)-1]*T,zeta,'-*')
hold off
legend('Zeta up','Zeta down','Zeta','Location','southeast')

%% Plot results COM frontal
figure(11)
clf
title('COM frontal')
xlabel('y [m]') % x-axis label
ylabel('z [m]') % y-axis label
axis equal
% axis image
hold on
plot(yc(no_begin+1:vcom_change),zc(no_begin+1:vcom_change),'*b')
h1=plot(yc_discret(no_begin*20+1:vcom_change*20),zc_discret(no_begin*20+1:vcom_change*20),'b');

plot(yc(vcom_change+1:(length(xvcom_ref)-no_end)),zc(vcom_change+1:(length(xvcom_ref)-no_end)),'*r')
h2=plot(yc_discret(vcom_change*20+1:(length(xvcom_ref)-no_end)*20),zc_discret(vcom_change*20+1:(length(xvcom_ref)-no_end)*20),'r');

hold off
legend([h1,h2],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1']},'Location','southeast')

%% Plot results COM sagittal
switch kinematic_limit
    case ''
        xstep_sampling=Px_step_ref(1:end-15,:)*xstep;
        ystep_sampling=Px_step_ref(1:end-15,:)*ystep;
        zstep_sampling=zzmp_ref(1:end-15);

        xdiff_c_step=xc-xstep_sampling;
        ydiff_c_step=yc-ystep_sampling;


        a=tan(angle_successive(1:end-1)');
        b=[polyhedron_lim(1:end-1)./cos(angle_successive(1:end-1))]';

        z=[xdiff_c_step*a+repmat(b,size(xdiff_c_step,1),1) ydiff_c_step*a+repmat(b,size(ydiff_c_step,1),1)]+zstep_sampling;

        xstep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*xstep;
        ystep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*ystep;
        zstep_sampling=[zzmp_ref(1:2);zzmp_ref(1:end-17)];

        xdiff_c_step=xc-xstep_sampling;
        ydiff_c_step=yc-ystep_sampling;


        a=tan(angle_successive(1:end-1)');
        b=[polyhedron_lim(1:end-1)./cos(angle_successive(1:end-1))]';

        z=[z [xdiff_c_step*a+repmat(b,size(xdiff_c_step,1),1) ydiff_c_step*a+repmat(b,size(ydiff_c_step,1),1)]+zstep_sampling];

        zmax=min(z,[],2);

        figure(12)
        clf
        title('COM sagittal')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis equal
%         axis image
        hold on
        plot(xc(1:vcom_change),zc(1:vcom_change),'*b')
        h1=plot(xc_discret(1:vcom_change*20),zc_discret(1:vcom_change*20),'b');

        plot(xc(vcom_change+1:end),zc(vcom_change+1:end),'*r')
        h2=plot(xc_discret(vcom_change*20+1:end),zc_discret(vcom_change*20+1:end),'r');


        h3=plot(xc(1:120),zmax(1:120),'*-');
        h4=[];
        for i=1:size(xstep,1)-2
            h4=plot([-0.52:0.01:0.52]+xstep(i),[min([-0.52:0.01:0.52]'*a+repmat(b,size([-0.52:0.01:0.52],1),1),[],2) repmat(-polyhedron_lim(end),size([-0.52:0.01:0.52]',1))],'k');
        end

        hold off
        if isempty(h4)
            legend([h1,h2,h3],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height'},'Location','southeast')
        else
            legend([h1,h2,h3,h4(1)],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height','kinematic limits polyhedron'},'Location','southeast')
        end
    case 'hexagon'
        xstep_sampling=Px_step_ref(1:end-15,:)*xstep;
        ystep_sampling=Px_step_ref(1:end-15,:)*ystep;
        zstep_sampling=zzmp_ref(1:end-15);

        xdiff_c_step=xc-xstep_sampling;
        ydiff_c_step=yc-ystep_sampling;


        % a=tan(angle_successive(1:end-1)');
        % b=[polyhedron_lim(1:end-1)./cos(angle_successive(1:end-1))]';

        % z=[xdiff_c_step*a+repmat(b,size(xdiff_c_step,1),1) ydiff_c_step*a+repmat(b,size(ydiff_c_step,1),1)]+zstep_sampling;

        z=[];
        for i=1:size(plan_hexagon,2)
            for j=1:size(plan_hexagon{i},1)
                z=[z -(plan_hexagon{i}(j,1)*xdiff_c_step+plan_hexagon{i}(j,2)*ydiff_c_step+plan_hexagon{i}(j,4))/plan_hexagon{i}(j,3)];
            end
        end

        xstep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*xstep;
        ystep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*ystep;
        zstep_sampling=[zzmp_ref(1:2);zzmp_ref(1:end-17)];

        xdiff_c_step=xc-xstep_sampling;
        ydiff_c_step=yc-ystep_sampling;


        % a=tan(angle_successive(1:end-1)');
        % b=[polyhedron_lim(1:end-1)./cos(angle_successive(1:end-1))]';
        % 
        % z=[z [xdiff_c_step*a+repmat(b,size(xdiff_c_step,1),1) ydiff_c_step*a+repmat(b,size(ydiff_c_step,1),1)]+zstep_sampling];

        for i=1:size(plan_hexagon,2)
            for j=1:size(plan_hexagon{i},1)
                z=[z -(plan_hexagon{i}(j,1)*xdiff_c_step+plan_hexagon{i}(j,2)*ydiff_c_step+plan_hexagon{i}(j,4))/plan_hexagon{i}(j,3)];
            end
        end

        z=z+zstep_sampling;

        [zmax,indices]=min(z,[],2);

        figure(12)
        clf
        title('COM sagittal')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis equal
%         axis image
        hold on
        plot(xc(1:vcom_change),zc(1:vcom_change),'*b')
        h1=plot(xc_discret(1:vcom_change*20),zc_discret(1:vcom_change*20),'b');

        plot(xc(vcom_change+1:end),zc(vcom_change+1:end),'*r')
        h2=plot(xc_discret(vcom_change*20+1:end),zc_discret(vcom_change*20+1:end),'r');


        h3=plot(xc(1:120),zmax(1:120),'*-');
        h4=[];
        % for i=1:size(xstep,1)-2
        %     h4=plot([-0.52:0.01:0.52]+xstep(i),[min([-0.52:0.01:0.52]'*a+repmat(b,size([-0.52:0.01:0.52],1),1),[],2) repmat(-polyhedron_lim(end),size([-0.52:0.01:0.52]',1))],'k');
        % end
        h5=plot(pstep(:,1),h_com,'o');

        hold off
        if isempty(h4)
            legend([h1,h2,h3],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height'},'Location','southeast')
        else
            legend([h1,h2,h3,h4(1)],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height','kinematic limits polyhedron'},'Location','southeast')
        end
        
    case 'hexagonTranslation'
%         translation_y=ystep_l_0*1;
        
        xstep_sampling=Px_step_ref(1:end-15,:)*xstep;
        ystep_sampling=Px_step_ref(1:end-15,:)*ystep;
        zstep_sampling=zzmp_ref(1:end-15);
%         xstep_sampling=Px_step_ref(2:end-14,:)*xstep;
%         ystep_sampling=Px_step_ref(2:end-14,:)*ystep;
%         zstep_sampling=zzmp_ref(2:end-14);

        xdiff_c_step=xc-(xstep_sampling+translation_x);
        ydiff_c_step=yc-(ystep_sampling-(sign(ystep_sampling))*translation_y);

        z=[];
        for i=1:size(plan_hexagon,2)
            for j=1:size(plan_hexagon{i},1)
                z=[z -(plan_hexagon{i}(j,1)*xdiff_c_step+plan_hexagon{i}(j,2)*ydiff_c_step+plan_hexagon{i}(j,4))/plan_hexagon{i}(j,3)];
            end
        end

        xstep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*xstep;
        ystep_sampling=[Px_step_ref(1:2,:);Px_step_ref(1:end-17,:)]*ystep;
        zstep_sampling=[zzmp_ref(1:2);zzmp_ref(1:end-17)];

        xdiff_c_step=xc-(xstep_sampling+translation_x);
        ydiff_c_step=yc-(ystep_sampling-(sign(ystep_sampling))*translation_y);

%         for i=1:size(plan_hexagon,2)
%             for j=1:size(plan_hexagon{i},1)
%                 z=[z -(plan_hexagon{i}(j,1)*xdiff_c_step+plan_hexagon{i}(j,2)*ydiff_c_step+plan_hexagon{i}(j,4))/plan_hexagon{i}(j,3)];
%             end
%         end

        z=z+zstep_sampling;

        [zmax,indices]=min(z,[],2);

        figure(12)
        clf
        title('COM sagittal')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis equal
%         axis square
        hold on
        plot(xc(1:vcom_change),zc(1:vcom_change),'*b')
        h1=plot(xc_discret(1:vcom_change*20),zc_discret(1:vcom_change*20),'b');

        plot(xc(vcom_change+1:length(xvcom_ref)-no_end),zc(vcom_change+1:length(xvcom_ref)-no_end),'*r')
        h2=plot(xc_discret(vcom_change*20+1:(length(xvcom_ref)-no_end)*20),zc_discret(vcom_change*20+1:(length(xvcom_ref)-no_end)*20),'r');


        h3=plot(xc(1:length(xvcom_ref)-no_end),zmax(1:length(xvcom_ref)-no_end),'*-');
        h4=[];
        % for i=1:size(xstep,1)-2
        %     h4=plot([-0.52:0.01:0.52]+xstep(i),[min([-0.52:0.01:0.52]'*a+repmat(b,size([-0.52:0.01:0.52],1),1),[],2) repmat(-polyhedron_lim(end),size([-0.52:0.01:0.52]',1))],'k');
        % end
        h5=plot(pstep(:,1),h_com,'o');

        hold off
        if isempty(h4)
            legend([h1,h2,h3],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height'},'Location','southeast')
        else
            legend([h1,h2,h3,h4(1)],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height','kinematic limits polyhedron'},'Location','southeast')
        end
end


%%
%% Plot results COM velocity
figure(14)
clf
title('COM velocity along x-axis')
xlabel('t [s]') % x-axis label
ylabel('x com velocity [m.s^-1]') % y-axis label
hold on
plot([0:size(xdc,1)-1]*T,xdc)
plot([0:size(xvcom_ref,1)-1]*T,xvcom_ref)
temp_vcom=smooth(xvcom_ref,16);
plot([0:size(temp_vcom,1)-1]*T,temp_vcom)

hold off
legend('COM vel','COM vel ref','Location','southeast')
