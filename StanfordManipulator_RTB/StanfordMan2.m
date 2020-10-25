%Initializing DH Parameters of 6 links
L1 = Link([pi/6,    0,  0,  -pi/2,  0,  0], 'standard');
L2 = Link([pi/3,    5,  0,  pi/2,   0,  0], 'standard');
L3 = Link([0,       10, 0,  0,      1,  0], 'standard');
L4 = Link([pi/4,    0,  0,  -pi/2,  0,  0], 'standard');
L5 = Link([5*pi/12, 0,  0,  pi/2,   0,  0], 'standard');
L6 = Link([pi/2,    0,  0,  0,      0,  0], 'standard');
L = [L1 L2 L3 L4 L5 L6];
%Creating the robot arm
MyRobotArm = SerialLink(L,'name','ARM2');
%Defining limits of every joint (min and max)
qlim    =   [0      pi/3;
    0      pi/2;
    10     15;
    0      pi/3;
    pi/3   pi;
    pi/2   pi];
MyRobotArm.qlim = qlim;
%Forward Kinematics Solution
%Position 0 - Joint Variables
q0      =   [pi/6   pi/3    10  pi/4    5*pi/12 pi/2];
%Position 0 - Joint angles
theta0  =   [pi/6   pi/3    0   pi/4    5*pi/12 pi/2];
%Position 0 - Joint d
d0      =   [L1.d   L2.d    q0(3)  L4.d    L5.d    L6.d];
%Position 0 - Joint twist angles
alpha0  =   [L1.alpha   L2.alpha    L3.alpha    L4.alpha    L5.alpha    L6.alpha];
%Calculating Transformation Matrix T06_0
T06_0 = FK(6,theta0,d0,alpha0);
%Ploting the RobotArm
%MyRobotArm.plot(q0);
%Inverse Kinematics Solution
%Example Transformation Matrix T06_Input
%Input Variables [pi/6, pi/3, 10, pi/4, 5*pi/12, pi/2]
T06_Input = [-0.6597   0.7367  0.1484  5;
    0.4356    0.2140  0.8743  8.6603;
    0.6124    0.6415  -0.4621 5;
    0         0       0       1];
InvKin1 = IK(T06_Input,qlim,L2.d);
%Testing IK against FK - 10 times
TestInputs = zeros(3125,6);
TestOutputs = zeros(3125,6);
Error =  zeros(3125,6);
ThetaSteps = zeros(5,5);
ThetaSteps(1,:) =[0,pi/6,pi/4,pi/3,pi/3];
ThetaSteps(2,:) =[0,pi/6,pi/4,pi/3,pi/2];
ThetaSteps(3,:) =[0,pi/6,pi/4,pi/3,pi/3];
ThetaSteps(4,:) =[pi/3,pi/2,2*pi/3,3*pi/4,pi-0.001];
ThetaSteps(5,:) =[pi/2,2*pi/3,3*pi/4,pi-0.00001,pi-0.001];
x = 1;
for a1=1:5
    TestInputs(x:625*a1,1) = ThetaSteps(1,a1);
    x = 625*a1+1;
end
x = 1;
for y=1:5
    for a2=1:5
        TestInputs(x:x+125,2) = ThetaSteps(2,a2);
        x = x+125+1;
    end
end
x=1;
for y=1:25
    for a3=1:5
        TestInputs(x:x+25,4) = ThetaSteps(3,a3);
        x = x+25+1;
    end
end
x=1;
for y =1:125
    for a4=1:5
        TestInputs(x:x+5,5) = ThetaSteps(4,a4);
        x =x+5+1;
    end
end
x=1;
for y=1:625
    for a5=1:5
        TestInputs(x,6) = ThetaSteps(5,a5);
        x =x+1;
    end
end
%TestInputs(:,3) = d0(3);
for i=1:3125
    T06_0 = FK(6,TestInputs(i,:),d0,alpha0);
    TestOutputs(i,:) = IK(T06_0,qlim,L2.d);
    Error(i,:) = TestOutputs(i,:) - TestInputs(i,:);
    Error(i,3) = TestOutputs(i,3) - d0(3);
end
for i=1:6
    subplot(2,3,i);plot(Error(:,i));
end


