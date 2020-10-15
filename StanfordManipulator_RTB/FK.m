function [OutT] = FK(n,InAngles,InD, InAlpha)
OutT = [cos(InAngles(1))    -sin(InAngles(1))*cos(InAlpha(1))   sin(InAngles(1))*sin(InAlpha(1))    0;
        sin(InAngles(1))    cos(InAngles(1))*cos(InAlpha(1))    -cos(InAngles(1))*sin(InAlpha(1))   0;
        0                   sin(InAlpha(1))                     cos(InAlpha(1))                     InD(1);
        0                   0                                   0                                   1];
%Calculating Transformation Matrix T06_0
 for i=2:n
    temp = [cos(InAngles(i))    -sin(InAngles(i))*cos(InAlpha(i))   sin(InAngles(i))*sin(InAlpha(i))    0;
            sin(InAngles(i))    cos(InAngles(i))*cos(InAlpha(i))    -cos(InAngles(i))*sin(InAlpha(i))   0;
            0                   sin(InAlpha(i))                     cos(InAlpha(i))                     InD(i);
            0                   0                                   0                                   1];
    OutT = OutT*temp;
 end
end

