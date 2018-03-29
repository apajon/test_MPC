%% polyhedron
angle_tot=acos((h_com+h_com_min)/(h_com+h_com_max));
angle_divide=angle_tot/3;

angle_successive=[-2*angle_divide;-angle_divide;0;angle_divide;2*angle_divide;pi];
rot_successive=[sin(angle_successive) cos(angle_successive)];

polyhedron_lim=[repmat((h_com+h_com_max),5,1);-(h_com+h_com_min)];