function [OutResult] = IK(InTMat,InLim, Ind2)
%T =    u_x ..  v_x ..  w_x ..  p_x
%       u_y ..  v_y ..  w_y ..  p_y
%       u_z ..  v_z ..  w_z ..  p_z
%       0   ..  0   ..  0   ..  1
OutResult = zeros(6,1);
u_x = InTMat(1,1);
u_y = InTMat(2,1);
u_z = InTMat(3,1);
v_x = InTMat(1,2);
v_y = InTMat(2,2);
v_z = InTMat(3,2);
w_x = InTMat(1,3);
w_y = InTMat(2,3);
w_z = InTMat(3,3);
p_x = InTMat(1,4);
p_y = InTMat(2,4);
p_z = InTMat(3,4);
r =  sqrt(p_x^2 + p_y^2);
OutResult(1) = atan2(p_y,p_x) - atan2(Ind2,sqrt(r^2-(Ind2)^2));
if(OutResult(1) >(InLim(1,2)+0.001) || OutResult(1)<(InLim(1,1)-0.001))
    OutResult(1) = atan2(p_y,p_x) - atan2(Ind2,-sqrt(r^2-(Ind2)^2));
end
c_1 = cos(OutResult(1));
s_1 = sin(OutResult(1));
OutResult(2) = atan2(c_1*p_x + s_1*p_y, p_z);
s_2 = sin(OutResult(2));
c_2 = cos(OutResult(2));
OutResult(3) = (p_x *c_1 + s_1*p_y)*(s_2) + p_z*c_2;
term_1 =  -s_1*w_x + c_1*w_y;
term_2 = c_2*(c_1*w_x + s_1*w_y) - s_2*w_z;
OutResult(4) = atan2(term_1,term_2);
c_4 = cos(OutResult(4));
s_4 = sin(OutResult(4));
term_1 = c_4*(c_2*(c_1*w_x+s_1*w_y)-s_2*w_z)+ ...
         s_4*(c_1*w_y -s_1*w_x);
term_2 = s_2*(c_1*w_x +s_1*w_y) +c_2*w_z;
OutResult(5) = atan2(term_1,term_2);
I = c_1*v_x + s_1*v_y;
nterm = -s_1*v_x + c_1*v_y;
s_5 = sin(OutResult(5));
c_5 = cos(OutResult(5));
term_1 = -c_5*(c_4*(c_2*I-s_2*v_z)+s_4*nterm)+ ...
         s_5*(s_2*I +c_2*v_z);
term_2 = -s_4*(c_2*I -s_2*v_z)+c_4*nterm;
OutResult(6) = atan2(term_1,term_2);
end

