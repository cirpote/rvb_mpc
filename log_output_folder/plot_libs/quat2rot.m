function R = quat2rot( q )
%QUAT2ROT Summary of this function goes here
%   Detailed explanation goes here
    qx = q(1);
    qy = q(2);
    qz = q(3);

    qw = 1 - (qx*qx + qy*qy + qz*qz);
    R = quat2rotm([qw, qx, qy, qz]);

end
