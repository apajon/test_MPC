%% Plot results top view
figure(15)
clf

title('trajectories 3D')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
axis equal
axis([pstep(4,1) pstep(8,1) (pstep(4,1)-pstep(8,1))/2 (pstep(8,1)-pstep(4,1))/2])
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

hold off
legend('COM','CoP','CoP up','CoP down','Location','southeast')

%% Plot results top view
figure(16)
clf

title('trajectories 3D')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
axis equal
axis([pstep(size(pstep,1)/2+3,1)-(pstep(8,1)-pstep(4,1))/2 pstep(size(pstep,1)/2+3,1)+(pstep(8,1)-pstep(4,1))/2 (pstep(4,1)-pstep(8,1))/2 (pstep(8,1)-pstep(4,1))/2])
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

hold off
legend('COM','CoP','CoP up','CoP down','Location','southeast')

%% Plot results COM sagittal
switch kinematic_limit
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

        for i=1:size(plan_hexagon,2)
            for j=1:size(plan_hexagon{i},1)
                z=[z -(plan_hexagon{i}(j,1)*xdiff_c_step+plan_hexagon{i}(j,2)*ydiff_c_step+plan_hexagon{i}(j,4))/plan_hexagon{i}(j,3)];
            end
        end

        z=z+zstep_sampling;

        [zmax,indices]=min(z,[],2);

        figure(17)
        clf
        title('COM sagittal')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis equal
        axis([pstep(4,1) pstep(8,1) 0.78-(pstep(8,1)-pstep(4,1))/2 0.78+(pstep(8,1)-pstep(4,1))/2])
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
        
        figure(18)
        clf
        title('COM sagittal')
        xlabel('x [m]') % x-axis label
        ylabel('z [m]') % y-axis label
        axis equal
        axis([pstep(4,1) pstep(8,1) 0.78-(pstep(8,1)-pstep(4,1))/2 0.78+(pstep(8,1)-pstep(4,1))/2])
        axis([pstep(size(pstep,1)/2-1,1) pstep(size(pstep,1)/2+4,1) 0.78-(pstep(size(pstep,1)/2+4,1)-pstep(size(pstep,1)/2-1,1))/2 0.78+(pstep(size(pstep,1)/2+4,1)-pstep(size(pstep,1)/2-1,1))/2])
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