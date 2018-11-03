%% Implementation by Nicolas Soncini - 2018
%% Following couse ws13 from freigburg uni

%% v2t takes as input the vector form of the robot
%% pose and outputs the corresponding homogeneous
%% trasnformation

1;

function [M] = v2t(v)

  M = [cos(v(3)), -sin(v(3)), v(1);
       sin(v(3)), cos(v(3)) , v(2);
       0        , 0         , 1   ];

end


%% t2v takes as input an homogeneous transformation
%% representing the robot pose in 2D and outputs the
%% corresponding compact vector
function [v] = t2v(M)

  v = [M(1,3), M(2,3), normalize_angle(acos(M(1,1)))];

end

%% p2rt takes as input two robot poses as vectors
%% and returns the relative transformation from
%% the first to the second
function [R] = relative_pos(p1,p2)

  R = inv(v2t(p1)) * v2t(p2);

end 

%% pos2land takes as input a robot pose and a landmark
%% position relative to it and returns the real location
%% of the landmark
function [lmark] = pos2land(p1,rel_lmark)

  lmark = 0;

end
