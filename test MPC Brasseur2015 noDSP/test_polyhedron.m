% h_com_max=+0.05;
% h_com_min=-0.2;

angle_tot=acos((h_com+h_com_min)/(h_com+h_com_max));
angle_divide=angle_tot/3;

angle_successive=[-2*angle_divide;-angle_divide;0;angle_divide;2*angle_divide;pi];
rot_successive=[sin(angle_successive) cos(angle_successive)];

polyhedron_lim=[repmat((h_com+h_com_max),5,1);-(h_com+h_com_min)];

foot_step_1=[backtoankle 0];
foot_step_2=[-fronttoankle 0];

foot_step_1=[0.25 0.14];
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

