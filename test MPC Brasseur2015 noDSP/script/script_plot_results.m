%% Plot results
figure(7)
clf

ax1 = subplot(2,1,1);
title('trajectory of COM 3d')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
view(3)
hold on
plot3(xc,yc,zc,'-*k')
hold off
legend('COM','Location','southeast')

ax2 = subplot(2,1,2);
title('trajectories 3d')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
view(3)
hold on
plot3(xz,yz,zz,'-*g')

plot3(xz_up,yz_up,zz_up,'-*r')
plot3(xz_down,yz_down,zz_down,'-*m')

plot3(xcapture,ycapture,zcapture,'-*b')
hold off
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

XY=drawing_rectangle_rotate(pstep,psi,backtoankle+4*sole_margin,fronttoankle+4*sole_margin,exttoankle+4*sole_margin,inttoankle+4*sole_margin,firstSS);
for j=1:length(pstep)
    fill3(XY(j,1:5),XY(j,6:10),repmat(zstep(j),1,5),[132,200,225]/255,'LineStyle','none')
end
hold off
hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'}); 
rotate3d on

legend('CoP','CoP up','CoP down','Capture point','Location','southeast')