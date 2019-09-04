function T = v2t( v )
%V2T Summary of this function goes here
%   Detailed explanation goes here

    T = eye(4);
    T(1:3,4) = v(1:3);
    T(1:3,1:3) = quat2rot(v(4:6));

end