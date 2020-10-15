clear all;
L1 = Link([0,       0.762,      0,      -pi/2,  0,  0], 'standard');
L2 = Link([-pi/2,   0.39412,    0,      -pi/2,  0,  0], 'standard');
L3 = Link([-pi/2,   0.635,      0,      0,      1,  0], 'standard');
L4 = Link([0,       0.2268,     0,      -pi/2,  0,  0], 'standard');
L5 = Link([pi,      0,          0,      -pi/2,  0,  0], 'standard');
L6 = Link([pi,      0.4318,     0,      0,      0,  0], 'standard');
L = [L1 L2 L3 L4 L5 L6];
MyRobotArm = SerialLink(L,'name','ARM')
qlim    =   [0          pi/3;           -pi/2       -5*pi/6;    0.635       0.735;0     pi/3;pi     5*pi/6;pi   5*pi/6];
q0      =   [0          -pi/2           0.635       0           pi          pi];
q1      =   [20*pi/180  -110*pi/180     0.66        20*pi/180   170*pi/180  170*pi/180];
theta0  =   [20*pi/180  -110*pi/180     -pi/2       20*pi/180   170*pi/180  170*pi/180];
alpha0  =   [L1.alpha   L2.alpha        L3.alpha    L4.alpha    L5.alpha    L6.alpha];
d0      =   [L1.d       L2.d            0.66        L4.d        L5.d        L6.d];
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
