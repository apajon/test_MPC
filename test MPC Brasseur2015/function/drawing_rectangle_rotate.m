function [XY]=drawing_rectangle_rotate(pstep,theta,backtoankle,fronttoankle,exttoankle,inttoankle,firstSS)
% %RECTANGLE('Position', [x y w h])
% w=3; %width
% h=2; %height
% x=0.2;y=1; %corner position
% xv=[x x+w x+w x x];yv=[y y y+h y+h y];
% figure(1), plot(xv,yv);axis equal;
% 
% %rotate angle alpha
% R(1,:)=xv;R(2,:)=yv;
% alpha=30*2*pi/360;
% XY=[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)]*R;
% hold on;plot(XY(1,:),XY(2,:),'r');
% axis([-2 4 0 5])

% Your polygon xv,yv is rotated around the origin, but you can
% translate xv,yv first if you want to rotate around some other point.

footlength=fronttoankle+backtoankle;
footwidth=exttoankle+inttoankle;

%RECTANGLE('Position', [x y w h])
w=footlength; %width
h=footwidth; %height
x=-backtoankle;
XY=zeros(length(pstep),10);

for i=1:length(pstep)
    switch (firstSS)
        case 0
            y=-inttoankle*mod(i+1,2)-exttoankle*mod(i,2); %corner position
        case 1
            y=-inttoankle*mod(i,2)-exttoankle*mod(i+1,2); %corner position
    end
    xv=[x x+w x+w x x];yv=[y y y+h y+h y];
    R=[xv;yv];
    if(i==1||i==2)
        alpha=theta(i);
    elseif(i==length(pstep))
        alpha=theta(end);
    else
        alpha=theta(3*(i-2)+1);
    end
    XY_=[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)]*R;
    XY(i,:)=[XY_(1,:)+pstep(i,1) XY_(2,:)+pstep(i,2)];
end

end