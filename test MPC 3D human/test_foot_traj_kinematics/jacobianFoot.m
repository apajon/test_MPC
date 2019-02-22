function [M]=jacobianFoot(q1,q2,R1,R2)

M=[-R1*sin(q1)-R2*sin(q1+q2) -R2*sin(q1+q2);
    R1*cos(q1)+R2*cos(q1+q2) R2*cos(q1+q2)];

end