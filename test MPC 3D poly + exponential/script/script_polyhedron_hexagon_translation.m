%% Polyhedron from hexagone decalage COM
z_ankle=0.093;
% z_ankle=0;
z_diff_c_hip=0.00;
z_decalage_tot=z_ankle+z_diff_c_hip;
z_leg_max=h_com+h_com_max-z_decalage_tot;
z_leg_min=z_leg_max-(h_com_max-h_com_min);



angle_tot_max=acos(z_leg_min/z_leg_max);
radius_max=sin(angle_tot_max)*z_leg_max;
%% compute hexagon vertex
p_sommet=[];
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

%% compute edge mesh of hexagon with the next one
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

%% compute the normal vector to each polyhedron face
normal{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        normal{j}(k,1:3)=cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6))/norm(cross(edge{j}(k,1:3)-edge{j}(k,4:6),edge{j}(k,7:9)-edge{j}(k,4:6)));
    end
    normal{j}=normal{j}.*repmat(sign(normal{j}(:,3)),1,3);
end

%% compute the plan equation coefficient (a,b,c,d) as ax+by+cy+d=0
plan_hexagon{1}=[];
for j=1:number_level-1
    for k=1:size(edge{j},1)
        plan_hexagon{j}(k,1:4)=[normal{j}(k,1:3) -dot(normal{j}(k,1:3),edge{j}(k,4:6))];
    end
end

% %compute the projection of the foot step (0,0,0) on each plan related to
% %each faces of the polyhedron
% projection{1}=[];
% for j=1:number_level-1
%     for k=1:size(edge{j},1)
%         toto=plan_hexagon{j}(k,4)/(sum((plan_hexagon{j}(k,1:3)).^2));
%         projection{j}(k,1:3)=[-toto*plan_hexagon{j}(k,1) -toto*plan_hexagon{j}(k,2) -toto*plan_hexagon{j}(k,3)];
%     end
% end
% 
% angle_successive_z{1}=[];
% angle_successive_{1}=[];
% for j=1:number_level-1
%     for k=1:size(edge{j},1)
% %         angle_successive_z{j}(k,1)=acos(dot([0 1],projection{j}(k,1:2))/norm(projection{j}(k,1:2)))*(-sign(projection{j}(k,1)));
%         angle_successive_z{j}(k,1)=acos(projection{j}(k,2)/norm(projection{j}(k,1:2)))*(sign(projection{j}(k,1)));
%     end
% end
% 
% [x,y,z] = sphere(100);
% figure(15)
% clf
% view(3)
% axis equal
% title('polyhedron made of faces from hexagon')
% xlabel('x [m]') % x-axis label
% ylabel('y [m]') % y-axis label
% zlabel('z [m]') % z-axis label
% hold on
% surf(x*z_leg_max+0, y*z_leg_max+0, z*z_leg_max+z_decalage_tot,'FaceAlpha',0.5,'EdgeColor','none');
% plot3(p_sommet{1}(1),p_sommet{1}(2),p_sommet{1}(3),'*b')
% for j=2:number_level
%     plot3(p_sommet{j}(1:3:end),p_sommet{j}(2:3:end),p_sommet{j}(3:3:end),'-*b')
% end
% 
% for j=1:number_level-1
%     for k=1:size(edge{j},1)
%          plot3(edge{j}(k,1:3:end),edge{j}(k,2:3:end),edge{j}(k,3:3:end),'-r')
%     end
% end
% 
% % %normal vector of each face
% % for j=1:number_level-1
% %     for k=1:size(normal{j},1)
% %         quiver3(edge{j}(k,4),edge{j}(k,5),edge{j}(k,6),normal{j}(k,1),normal{j}(k,2),normal{j}(k,3))
% %     end
% % end
% 
% % %projection of origin on each polyhedron face
% % for j=1:number_level-1
% %     for k=1:size(normal{j},1)
% %         plot3(projection{j}(k,1),projection{j}(k,2),projection{j}(k,3),'*r')
% %     end
% % end
% hold off