function [angles angles2] = R2euler(R,m)
    ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

    b2 = subprob4(ey, ez, ex, ez'*R*ex); %two solutions!
    b22 = b2(2);
    b2 = b2(1);
    
    b1 = subprob1(-ez,R*ex,rotatoy(b2)*ex);
    b3 = subprob1(ex,R'*ez,rotatoy(-b2)*ez);
    

    b11 = subprob1(-ez,R*ex,rotatoy(b22)*ex);
    b33 = subprob1(ex,R'*ez,rotatoy(-b22)*ez);
    
    angles = [b1 b2 b3];
    angles2 = [b11 b22 b33];
end

function Ry = rotatoy(theta)
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end


