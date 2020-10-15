clear all;
L1 = Link([pi/6,    0,  0,  -pi/2,  0,  0], 'standard');
L2 = Link([pi/3,    5,  0,  pi/2,   0,  0], 'standard');
L3 = Link([0,       10, 0,  0,      1,  0], 'standard');
L4 = Link([pi/4,    0,  0,  -pi/2,  0,  0], 'standard');
L5 = Link([5*pi/12, 0,  0,  pi/2,   0,  0], 'standard');
L6 = Link([pi/2,    0,  0,  0,      0,  0], 'standard');
L = [L1 L2 L3 L4 L5 L6];
MyRobotArm = SerialLink(L,'name','ARM2');
qlim    =   [0          pi/3;           0           pi/2;       10          15;0     pi/3;pi/3     pi;pi/2   pi];
q0      =   [pi/6       pi/3            10          pi/4        5*pi/12     pi/2];
theta0  =   [pi/6       pi/3            0           pi/4        5*pi/12     pi/2];
alpha0  =   [L1.alpha   L2.alpha        L3.alpha    L4.alpha    L5.alpha    L6.alpha];
d0      =   [L1.d       L2.d            10          L4.d        L5.d        L6.d];
MyRobotArm.qlim = qlim;
T = [cos(theta0(1))  -sin(theta0(1))*cos(alpha0(1))  sin(theta0(1))*sin(alpha0(1))   0;
     sin(theta0(1))  cos(theta0(1))*cos(alpha0(1))   -cos(theta0(1))*sin(alpha0(1))  0;
     0               sin(alpha0(1))                  cos(alpha0(1))                  d0(1);
     0               0                               0                               1];
for i=2:6
    temp = [cos(theta0(i))  -sin(theta0(i))*cos(alpha0(i))  sin(theta0(i))*sin(alpha0(i))   0;
            sin(theta0(i))  cos(theta0(i))*cos(alpha0(i))   -cos(theta0(i))*sin(alpha0(i))  0;
            0               sin(alpha0(i))                  cos(alpha0(i))                  d0(i);
            0               0                               0                               1];
    T = T*temp;
end
%MyRobotArm.plot(q0);
InvKin = zeros(6,2);
T2 = [-0.6597   0.7367  0.1484  5;
      0.4356    0.2140  0.8743  8.6603;
      0.6124    0.6415  -0.4621 5;
      0         0       0       1]
r =  sqrt((T2(1,4))^2 + (T2(2,4))^2);
InvKin(1,1) = atan2(T2(2,4),T2(1,4)) - atan2(L2.d,sqrt(r^2-(L2.d)^2));
InvKin(1,2) = atan2(T2(2,4),T2(1,4)) - atan2(L2.d,-sqrt(r^2-(L2.d)^2));
