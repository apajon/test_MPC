% h_com_max=+0.05;
% h_com_min=-0.2;

angle_tot=acos((h_com+h_com_min)/(h_com+h_com_max));
angle_divide=angle_tot/3;

angle_successive=[-2*angle_divide;-angle_divide;0;angle_divide;2*angle_divide;pi];
rot_successive=[sin(angle_successive) cos(angle_successive)];

polyhedron_lim=[repmat((h_com+h_com_max),5,1);-(h_com+h_com_min)];

foot_step_1=[backtoankle 0];
foot_step_2=[-fronttoankle 0];

foot_step_1=[0.25 0.18];
foot_step_2=[-0.25 0];

figure(6)
clf
axis equal
hold on
plot([foot_step_1(1) foot_step_2(1)],[foot_step_1(2) foot_step_2(2)],'o')

plot([0.6 -0.6],[h_com+h_com_max h_com+h_com_max]+foot_step_1(2))
plot([0.6 -0.6],[h_com+h_com_min h_com+h_com_min]+foot_step_1(2))

plot([0.6 -0.6],[h_com+h_com_max h_com+h_com_max]+foot_step_2(2))
plot([0.6 -0.6],[h_com+h_com_min h_com+h_com_min]+foot_step_2(2))


% plot((h_com+h_com_max)*sin(-pi/4:(pi/4)/20:pi/4)+0.20,(h_com+h_com_max)*cos(-pi/4:(pi/4)/20:pi/4))
% plot((h_com+h_com_max)*sin(-pi/4:(pi/4)/20:pi/4)-0.20,(h_com+h_com_max)*cos(-pi/4:(pi/4)/20:pi/4))

for j=[1 2 4 5]
plot(cos(angle_successive(j))*[-0.2 0.2]-sin(angle_successive(j))*(h_com+h_com_max)+foot_step_1(1),sin(angle_successive(j))*[-0.2 0.2]+cos(angle_successive(j))*(h_com+h_com_max)+foot_step_1(2),'k')
plot(cos(angle_successive(j))*[-0.2 0.2]-sin(angle_successive(j))*(h_com+h_com_max)+foot_step_2(1),sin(angle_successive(j))*[-0.2 0.2]+cos(angle_successive(j))*(h_com+h_com_max)+foot_step_2(2),'b')
end
hold off

%% Polyhedron from hexagone
clear p_sommet
angle_tot_max=acos((h_com+h_com_min)/(h_com+h_com_max));
radius_max=sin(angle_tot_max)*(h_com+h_com_max);
number_level=3;

p_sommet=[];
%compute hexagon vertex
for j=1:number_level
    if j==1
        p_sommet{1}=[0 0 h_com+h_com_max];
    else
%         radius=2^(j-1)*radius_max/2^(number_level-1);
        radius=(j-1)*radius_max/(number_level-1);
        angle_tot=asin(radius/(h_com+h_com_max));
        
        height=cos(angle_tot)*(h_com+h_com_max);
        
        for k=1:6
%             p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)=[radius*cos(k*pi/3) radius*sin(k*pi/3)  height];
            p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)=[radius*cos(k*pi/3) radius*sin(k*pi/3)  sqrt((h_com+h_com_max)^2-(radius*cos(k*pi/3))^2-(radius*sin(k*pi/3))^2)];
        end
%         p_sommet{j}(1,(7-1)*3+1:(7-1)*3+3)=[radius*cos(7*pi/3) radius*sin(7*pi/3)  height];
        p_sommet{j}(1,(7-1)*3+1:(7-1)*3+3)=[radius*cos(7*pi/3) radius*sin(7*pi/3)  sqrt((h_com+h_com_max)^2-(radius*cos(7*pi/3))^2-(radius*sin(7*pi/3))^2)];
        
        p_sommet_extend=[];
        if j>2
            for k=1:6
%                 p_sommet_extend=[p_sommet_extend p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3) ...
%                     (p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)+p_sommet{j}(1,(k)*3+1:(k)*3+3))/2];
                p_sommet_extend=[p_sommet_extend p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3) ...
                    (p_sommet{j}(1,(k-1)*3+1:(k-1)*3+2)+p_sommet{j}(1,(k)*3+1:(k)*3+2))/2 ...
                    sqrt((h_com+h_com_max)^2-sum( ((p_sommet{j}(1,(k-1)*3+1:(k-1)*3+2)+p_sommet{j}(1,(k)*3+1:(k)*3+2))/2).^2 ))];
            end
            p_sommet_extend=[p_sommet_extend p_sommet{j}(end-2:end)];
            p_sommet{j}=p_sommet_extend;
        end
    end
end

%compute edge mesh of hexagon with the next one
edge{1}=[];
for j=1:number_level-1
    if j==1
        for k=1:6
            edge{1}=[edge{1};...
                p_sommet{2}((k-1)*3+1:(k-1)*3+3) p_sommet{1}(1:3) p_sommet{2}((k)*3+1:(k)*3+3)];
        end
    elseif j==2
        for k=1:6
            edge{2}((k-1)*3+1:(k-1)*3+3,1:9)=[p_sommet{3}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{2}(1,(k-1)*3+1:(k-1)*3+3) p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6);...
                p_sommet{2}(1,(k-1)*3+1:(k-1)*3+3) p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{2}(1,(k)*3+1:(k)*3+3);...
                p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{2}(1,(k)*3+1:(k)*3+3) p_sommet{3}(1,(k-1)*3*2+7:(k-1)*3*2+9)];
        end
    else
        for k=1:6
            edge{j}((k-1)*3+1:(k-1)*3+3,1:9)=[p_sommet{j+1}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6);...
                p_sommet{j}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{j}(1,(k-1)*3*2+7:(k-1)*3*2+9);...
                p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{j}(1,(k-1)*3*2+7:(k-1)*3*2+9) p_sommet{j+1}(1,(k-1)*3*2+7:(k-1)*3*2+9)];
        end
    end
end

%compute the normal vector to each polyhedron face
normal{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        normal{j}(k,1:3)=cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6))/norm(cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6)));
    end
    normal{j}=normal{j}.*repmat(sign(normal{j}(:,3)),1,3);
end

%compute the plan equation coefficient (a,b,c,d) as ax+by+cy+d=0
plan_hexagon{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        plan_hexagon{j}(k,1:4)=[normal{j}(k,1:3) -dot(normal{j}(k,1:3),edge{j}(k,4:6))];
    end
end

%compute the projection of the foot step (0,0,0) on each plan related to
%each faces of the polyhedron
projection{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        toto=plan_hexagon{j}(k,4)/(sum((plan_hexagon{j}(k,1:3)).^2));
        projection{j}(k,1:3)=[-toto*plan_hexagon{j}(k,1) -toto*plan_hexagon{j}(k,2) -toto*plan_hexagon{j}(k,3)];
    end
end

angle_successive_z{1}=[];
angle_successive_{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
%         angle_successive_z{j}(k,1)=acos(dot([0 1],projection{j}(k,1:2))/norm(projection{j}(k,1:2)))*(-sign(projection{j}(k,1)));
        angle_successive_z{j}(k,1)=acos(projection{j}(k,2)/norm(projection{j}(k,1:2)))*(sign(projection{j}(k,1)));
    end
end

[x,y,z] = sphere(100);
figure(15)
clf
view(3)
axis equal
title('polyhedron made of faces from hexagon')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
hold on
surf(x*(h_com+h_com_max)+0, y*(h_com+h_com_max)+0, z*(h_com+h_com_max)+0,'FaceAlpha',0.5,'EdgeColor','none');
plot3(p_sommet{1}(1),p_sommet{1}(2),p_sommet{1}(3),'*b')
for j=2:number_level
    plot3(p_sommet{j}(1:3:end),p_sommet{j}(2:3:end),p_sommet{j}(3:3:end),'-*b')
end

for j=1:number_level-1
    for k=1:size(edge{j},1)
         plot3(edge{j}(k,1:3:end),edge{j}(k,2:3:end),edge{j}(k,3:3:end),'-r')
    end
end

% %normal vector of each face
% for j=1:number_level-1
%     for k=1:size(normal{j},1)
%         quiver3(edge{j}(k,4),edge{j}(k,5),edge{j}(k,6),normal{j}(k,1),normal{j}(k,2),normal{j}(k,3))
%     end
% end

% %projection of origin on each polyhedron face
% for j=1:number_level-1
%     for k=1:size(normal{j},1)
%         plot3(projection{j}(k,1),projection{j}(k,2),projection{j}(k,3),'*r')
%     end
% end
hold off

%% Polyhedron from hexagone decalage COM
clear p_sommet
% z_ankle=0.093;
% z_diff_c_hip=0.0;
z_decalage_tot=z_ankle+z_diff_c_hip;
z_leg_max=h_com+h_com_max-z_decalage_tot;
z_leg_min=z_leg_max-(h_com_max-h_com_min);



angle_tot_max=acos(z_leg_min/z_leg_max);
radius_max=sin(angle_tot_max)*z_leg_max;
number_level=2;

p_sommet=[];
%compute hexagon vertex
for j=1:number_level
    if j==1
        p_sommet{1}=[0 0 z_leg_max+z_decalage_tot];
    else
%         radius=2^(j-1)*radius_max/2^(number_level-1);
        radius=(j-1)*radius_max/(number_level-1);
        angle_tot=asin(radius/z_leg_max);
        
        height=cos(angle_tot)*z_leg_max;
        
        for k=1:6
%             p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)=[radius*cos(k*pi/3) radius*sin(k*pi/3)  height];
            p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)=[radius*cos(k*pi/3) radius*sin(k*pi/3)  sqrt(z_leg_max^2-(radius*cos(k*pi/3))^2-(radius*sin(k*pi/3))^2)+z_decalage_tot];
        end
%         p_sommet{j}(1,(7-1)*3+1:(7-1)*3+3)=[radius*cos(7*pi/3) radius*sin(7*pi/3)  height];
        p_sommet{j}(1,(7-1)*3+1:(7-1)*3+3)=[radius*cos(7*pi/3) radius*sin(7*pi/3)  sqrt(z_leg_max^2-(radius*cos(7*pi/3))^2-(radius*sin(7*pi/3))^2)+z_decalage_tot];
        
        p_sommet_extend=[];
        if j>2
            for k=1:6
%                 p_sommet_extend=[p_sommet_extend p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3) ...
%                     (p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3)+p_sommet{j}(1,(k)*3+1:(k)*3+3))/2];
                p_sommet_extend=[p_sommet_extend p_sommet{j}(1,(k-1)*3+1:(k-1)*3+3) ...
                    (p_sommet{j}(1,(k-1)*3+1:(k-1)*3+2)+p_sommet{j}(1,(k)*3+1:(k)*3+2))/2 ...
                    sqrt(z_leg_max^2-sum( ((p_sommet{j}(1,(k-1)*3+1:(k-1)*3+2)+p_sommet{j}(1,(k)*3+1:(k)*3+2))/2).^2 ))+z_decalage_tot];
            end
            p_sommet_extend=[p_sommet_extend p_sommet{j}(end-2:end)];
            p_sommet{j}=p_sommet_extend;
        end
    end
end

%compute edge mesh of hexagon with the next one
edge{1}=[];
for j=1:number_level-1
    if j==1
        for k=1:6
            edge{1}=[edge{1};...
                p_sommet{2}((k-1)*3+1:(k-1)*3+3) p_sommet{1}(1:3) p_sommet{2}((k)*3+1:(k)*3+3)];
        end
    elseif j==2
        for k=1:6
            edge{2}((k-1)*3+1:(k-1)*3+3,1:9)=[p_sommet{3}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{2}(1,(k-1)*3+1:(k-1)*3+3) p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6);...
                p_sommet{2}(1,(k-1)*3+1:(k-1)*3+3) p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{2}(1,(k)*3+1:(k)*3+3);...
                p_sommet{3}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{2}(1,(k)*3+1:(k)*3+3) p_sommet{3}(1,(k-1)*3*2+7:(k-1)*3*2+9)];
        end
    else
        for k=1:6
            edge{j}((k-1)*3+1:(k-1)*3+3,1:9)=[p_sommet{j+1}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6);...
                p_sommet{j}(1,(k-1)*3*2+1:(k-1)*3*2+3) p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{j}(1,(k-1)*3*2+7:(k-1)*3*2+9);...
                p_sommet{j+1}(1,(k-1)*3*2+4:(k-1)*3*2+6) p_sommet{j}(1,(k-1)*3*2+7:(k-1)*3*2+9) p_sommet{j+1}(1,(k-1)*3*2+7:(k-1)*3*2+9)];
        end
    end
end

%compute the normal vector to each polyhedron face
normal{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        normal{j}(k,1:3)=cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6))/norm(cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6)));
    end
    normal{j}=normal{j}.*repmat(sign(normal{j}(:,3)),1,3);
end

%compute the plan equation coefficient (a,b,c,d) as ax+by+cy+d=0
plan_hexagon{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        plan_hexagon{j}(k,1:4)=[normal{j}(k,1:3) -dot(normal{j}(k,1:3),edge{j}(k,4:6))];
    end
end

%compute the projection of the foot step (0,0,0) on each plan related to
%each faces of the polyhedron
projection{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        toto=plan_hexagon{j}(k,4)/(sum((plan_hexagon{j}(k,1:3)).^2));
        projection{j}(k,1:3)=[-toto*plan_hexagon{j}(k,1) -toto*plan_hexagon{j}(k,2) -toto*plan_hexagon{j}(k,3)];
    end
end

angle_successive_z{1}=[];
angle_successive_{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
%         angle_successive_z{j}(k,1)=acos(dot([0 1],projection{j}(k,1:2))/norm(projection{j}(k,1:2)))*(-sign(projection{j}(k,1)));
        angle_successive_z{j}(k,1)=acos(projection{j}(k,2)/norm(projection{j}(k,1:2)))*(sign(projection{j}(k,1)));
    end
end

[x,y,z] = sphere(100);
figure(15)
clf
view(3)
axis equal
title('polyhedron made of faces from hexagon')
xlabel('x [m]') % x-axis label
ylabel('y [m]') % y-axis label
zlabel('z [m]') % z-axis label
hold on
surf(x*z_leg_max+0, y*z_leg_max+0, z*z_leg_max+z_decalage_tot,'FaceAlpha',0.5,'EdgeColor','none');
% surf(x*(h_com+h_com_max)+0, y*(h_com+h_com_max)+0, z*(h_com+h_com_max)+0,'FaceAlpha',0.5,'EdgeColor','none');
plot3(p_sommet{1}(1),p_sommet{1}(2),p_sommet{1}(3),'*b')
for j=2:number_level
    plot3(p_sommet{j}(1:3:end),p_sommet{j}(2:3:end),p_sommet{j}(3:3:end),'-*b')
end

for j=1:number_level-1
    for k=1:size(edge{j},1)
         plot3(edge{j}(k,1:3:end),edge{j}(k,2:3:end),edge{j}(k,3:3:end),'-r')
    end
end

surf(x*z_leg_max+0.48, y*z_leg_max+0, z*z_leg_max+z_decalage_tot,'FaceAlpha',0.5,'EdgeColor','none');
plot3(p_sommet{1}(1)+0.48,p_sommet{1}(2)+0,p_sommet{1}(3),'*b')
for j=2:number_level
    plot3(p_sommet{j}(1:3:end)+0.48,p_sommet{j}(2:3:end)+0,p_sommet{j}(3:3:end),'-*b')
end

for j=1:number_level-1
    for k=1:size(edge{j},1)
         plot3(edge{j}(k,1:3:end)+0.48,edge{j}(k,2:3:end)+0,edge{j}(k,3:3:end),'-r')
    end
end
% %normal vector of each face
% for j=1:number_level-1
%     for k=1:size(normal{j},1)
%         quiver3(edge{j}(k,4),edge{j}(k,5),edge{j}(k,6),normal{j}(k,1),normal{j}(k,2),normal{j}(k,3))
%     end
% end

% %projection of origin on each polyhedron face
% for j=1:number_level-1
%     for k=1:size(normal{j},1)
%         plot3(projection{j}(k,1),projection{j}(k,2),projection{j}(k,3),'*r')
%     end
% end
hold off

%% Plot results COM sagittal
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
axis equal
title('kinematic limits polyhedron')
xlabel('x or y [m]') % x-axis label
ylabel('z [m]') % y-axis label
hold on
h1=plot(0,0,'o');


h4=plot([-0.52:0.01:0.52]+0,[min([-0.52:0.01:0.52]'*a+repmat(b,size([-0.52:0.01:0.52],1),1),[],2) repmat(-polyhedron_lim(end),size([-0.52:0.01:0.52]',1))],'k');
% h4=plot([-0.52 0.52]+0,[min([-0.52 0.52]'*a+repmat(b,size([-0.52 0.52],1),1),[],2) repmat(-polyhedron_lim(end),size([-0.52 0.52]',1))],'k');
hold off
legend('foot step position','kinematics limit polyhedron','Location','southeast')
% if isempty(h4)
%     legend([h1,h2,h3],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height'},'Location','southeast')
% else
%     legend([h1,h2,h3,h4(1)],{['COM wth spd ' num2str(vcom_1) 'm.s-1'],['COM with spd ' num2str(vcom_2) 'm.s-1'],'limit max COM height','kinematic limits polyhedron'},'Location','southeast')
% end

