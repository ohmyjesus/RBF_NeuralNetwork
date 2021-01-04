function [sys,x0,str,ts] = plant_puma_robot(t,x,u,flag)
% 以下程序为凯恩方法的六关节动力学建模
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}
    sys=[];
  case 3
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12;   %变量个数
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 12;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   
sys = simsizes(sizes);
x0  = [0 0 0 0 0 0 0 0 0 0 0 0];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(t,x,u)
%定义关节角和关节角速度x(1)~x(12)的代表含义
% 角度输入
q1=x(1);
q2=x(2);
q3=x(3);
q4=x(4);
q5=x(5);
q6=x(6);

% 角速度输入
dq1=x(7);
dq2=x(8);
dq3=x(9);
dq4=x(10);
dq5=x(11);
dq6=x(12);

q = [q1;q2;q3;q4;q5;q6];
dq = [dq1;dq2;dq3;dq4;dq5;dq6];
%创建V矩阵，需要用到连杆长度a质心位置r
%连杆长度
a1 = [0; 0; 0];
a2 = [-0.1573; 0.0653; 0];
a3 = [-0.0165; 0.0464; 0.6136];
a4 = [-0.2975; -0.0519; 0.1671];
a5 = [-0.3545; 0.0440; -0.0437];
a6 = [0.0820; -0.0145; -0.5149];

%各个连杆的相对前一个关节坐标系的重心即质心
r1 = [0.0467; -0.0104; 0.1100];
r2 = [-0.0155; -0.2895; -0.0268];
r3 = [0.1366; 0.1311; 0.0806];
r4 = [0.0350; 0.0044; 0.1474];
r5 = [3.9663e-06; -0.0019; 0.0540];
r6 = [0; 0; 0.05];

%连杆质量
%六个连杆的质量
m1 = 17.9513;
m2 = 3.5484;
m3 = 7.3201;
m4 = 3.8682;
m5 = 0.7287;
m6 = 1;
m=[m1;m2;m3;m4;m5;m6];

%各个连杆的转动惯量
J1 = [ 0.5354  0.0131  -0.2059;
       0.0131  0.7118   0.0404;
      -0.2059   0.0404  0.5010];
        
J2 = [0.5044  -0.0164  -0.0021;
    -0.0164, 0.0144  -0.0304;
    -0.0021, -0.0304  0.5091];
        
J3 = [0.2601, -0.1844  -0.0883;
    -0.1844, 0.2780  -0.0850;
    -0.0883, -0.0850  0.3979];
        
J4 = [0.1544  -0.0001  -0.0143;
    -0.0001  0.1527  -0.0051;
    -0.0143  -0.0051  0.0224];
        
J5 = [0.0055  0  0;
    0, 0.0040, -0.0015;
    0, -0.0015, 0.0028];
        
J6 = [0.0042  0  0;
        0  0.005  0;
        0  0  0.02];

%连杆扭转
alpha =  [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
theta = [x(1), x(2)+pi/2, x(3), x(4), x(5)+pi/2, x(6)];

%变换矩阵A的定义 先绕x旋转再绕z轴旋转
A01 = [1 0 0;0 cos(alpha(1)) -sin(alpha(1));0 sin(alpha(1)) cos(alpha(1))]* [cos(theta(1)) -sin(theta(1)) 0;sin(theta(1))  cos(theta(1)) 0;0 0 1];
A12 = [1 0 0;0 cos(alpha(2)) -sin(alpha(2));0 sin(alpha(2)) cos(alpha(2))]* [cos(theta(2)) -sin(theta(2)) 0;sin(theta(2))  cos(theta(2)) 0;0 0 1];
A23 = [1 0 0;0 cos(alpha(3)) -sin(alpha(3));0 sin(alpha(3)) cos(alpha(3))]* [cos(theta(3)) -sin(theta(3)) 0;sin(theta(3))  cos(theta(3)) 0;0 0 1];
A34 = [1 0 0;0 cos(alpha(4)) -sin(alpha(4));0 sin(alpha(4)) cos(alpha(4))]* [cos(theta(4)) -sin(theta(4)) 0;sin(theta(4))  cos(theta(4)) 0;0 0 1];
A45 = [1 0 0;0 cos(alpha(5)) -sin(alpha(5));0 sin(alpha(5)) cos(alpha(5))]* [cos(theta(5)) -sin(theta(5)) 0;sin(theta(5))  cos(theta(5)) 0;0 0 1];
A56 = [1 0 0;0 cos(alpha(6)) -sin(alpha(6));0 sin(alpha(6)) cos(alpha(6))]* [cos(theta(6)) -sin(theta(6)) 0;sin(theta(6))  cos(theta(6)) 0;0 0 1];

A1=A01;
A2=A12*A01;
A3=A23*A12*A01;
A4=A34*A23*A12*A01;
A5=A45*A34*A23*A12*A01;
A6=A56*A45*A34*A23*A12*A01;

%定义VT中的叉乘矩阵
xA1r1=A1*r1;
chachengxA1r1=[0 -xA1r1(3) xA1r1(2);xA1r1(3) 0 -xA1r1(1);-xA1r1(2) xA1r1(1) 0];
xA2r2=A2*r2;
chachengxA2r2=[0 -xA2r2(3) xA2r2(2);xA2r2(3) 0 -xA2r2(1);-xA2r2(2) xA2r2(1) 0];
xA3r3=A3*r3;
chachengxA3r3=[0 -xA3r3(3) xA3r3(2);xA3r3(3) 0 -xA3r3(1);-xA3r3(2) xA3r3(1) 0];
xA4r4=A4*r4;
chachengxA4r4=[0 -xA4r4(3) xA4r4(2);xA4r4(3) 0 -xA4r4(1);-xA4r4(2) xA4r4(1) 0];
xA5r5=A5*r5;
chachengxA5r5=[0 -xA5r5(3) xA5r5(2);xA5r5(3) 0 -xA5r5(1);-xA5r5(2) xA5r5(1) 0];
xA6r6=A6*r6;
chachengxA6r6=[0 -xA6r6(3) xA6r6(2);xA6r6(3) 0 -xA6r6(1);-xA6r6(2) xA6r6(1) 0];

%定义叉乘矩阵A1*qx
xA1a2=A1*a2;  %3*1
chachengxA1a2=[0 -xA1a2(3) xA1a2(2);xA1a2(3) 0 -xA1a2(1);-xA1a2(2) xA1a2(1) 0];  %3*3矩阵
xA2a3=A2*a3;
chachengxA2a3=[0 -xA2a3(3) xA2a3(2);xA2a3(3) 0 -xA2a3(1);-xA2a3(2) xA2a3(1) 0];
xA3a4=A3*a4;
chachengxA3a4=[0 -xA3a4(3) xA3a4(2);xA3a4(3) 0 -xA3a4(1);-xA3a4(2) xA3a4(1) 0];
xA4a5=A4*a5;
chachengxA4a5=[0 -xA4a5(3) xA4a5(2);xA4a5(3) 0 -xA4a5(1);-xA4a5(2) xA4a5(1) 0];
xA5a6=A5*a6;
chachengxA5a6=[0 -xA5a6(3) xA5a6(2);xA5a6(3) 0 -xA5a6(1);-xA5a6(2) xA5a6(1) 0];

%定义VkT  3*18矩阵
V1T=-[chachengxA1r1 [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0] ];
V2T=-[chachengxA1a2 chachengxA2r2 [0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0]];
V3T=-[chachengxA1a2 chachengxA2a3 chachengxA3r3 [0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0]];
V4T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4r4 [0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0]];
V5T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4a5 chachengxA5r5 [0 0 0;0 0 0;0 0 0]];
V6T=-[chachengxA1a2 chachengxA2a3 chachengxA3a4 chachengxA4a5 chachengxA5a6 chachengxA6r6];
V1=V1T';  %18*3矩阵
V2=V2T';
V3=V3T';
V4=V4T';
V5=V5T';
V6=V6T';

%定义W和WT
e3=[0;0;1];
W1=[e3'*A1';0 0 0;0 0 0;0 0 0;0 0 0;0 0 0];   %6*3矩阵
W2=[e3'*A1';e3'*A2';0 0 0;0 0 0;0 0 0;0 0 0];
W3=[e3'*A1';e3'*A2';e3'*A3';0 0 0;0 0 0;0 0 0];
W4=[e3'*A1';e3'*A2';e3'*A3';e3'*A4';0 0 0;0 0 0];
W5=[e3'*A1';e3'*A2';e3'*A3';e3'*A4';e3'*A5';0 0 0];
W6=[e3'*A1';e3'*A2';e3'*A3';e3'*A4';e3'*A5';e3'*A6'];
W1T=W1';    %3*6矩阵
W2T=W2';
W3T=W3';
W4T=W4';
W5T=W5';
W6T=W6';

%定义W
W=[W1 W2 W3 W4 W5 W6];          %6*18矩阵
WT=[W1T;W2T;W3T;W4T;W5T;W6T];   %18*6矩阵


%定义角速度
w1=A1*e3*dq(1)     ;              %3*1 矩阵
w2=A1*e3*dq(1)+A2*e3*dq(2);
w3=A1*e3*dq(1)+A2*e3*dq(2)+A3*e3*dq(3);
w4=A1*e3*dq(1)+A2*e3*dq(2)+A3*e3*dq(3)+A4*e3*dq(4);
w5=A1*e3*dq(1)+A2*e3*dq(2)+A3*e3*dq(3)+A4*e3*dq(4)+A5*e3*dq(5);
w6=A1*e3*dq(1)+A2*e3*dq(2)+A3*e3*dq(3)+A4*e3*dq(4)+A5*e3*dq(5)+A6*e3*dq(6);

%定义角速度的叉乘矩阵
xw1=[0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0];   %3*3矩阵
xw2=[0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0];
xw3=[0 -w3(3) w3(2);w3(3) 0 -w3(1);-w3(2) w3(1) 0];
xw4=[0 -w4(3) w4(2);w4(3) 0 -w4(1);-w4(2) w4(1) 0];
xw5=[0 -w5(3) w5(2);w5(3) 0 -w5(1);-w5(2) w5(1) 0];
xw6=[0 -w6(3) w6(2);w6(3) 0 -w6(1);-w6(2) w6(1) 0];

%定义dVkT中的叉乘矩阵
xxw1A1r1=xw1*A1*r1;   %3*1矩阵
chachengxxw1A1r1=[0 -xxw1A1r1(3) xxw1A1r1(2);xxw1A1r1(3) 0 -xxw1A1r1(1);-xxw1A1r1(2) xxw1A1r1(1) 0];
xxw1A1a2=xw1*A1*a2;
chachengxxw1A1a2=[0 -xxw1A1a2(3) xxw1A1a2(2);xxw1A1a2(3) 0 -xxw1A1a2(1);-xxw1A1a2(2) xxw1A1a2(1) 0];
xxw2A2r2=xw2*A2*r2;
chachengxxw2A2r2=[0 -xxw2A2r2(3) xxw2A2r2(2);xxw2A2r2(3) 0 -xxw2A2r2(1);-xxw2A2r2(2) xxw2A2r2(1) 0];
xxw2A2a3=xw2*A2*a3;
chachengxxw2A2a3=[0 -xxw2A2a3(3) xxw2A2a3(2);xxw2A2a3(3) 0 -xxw2A2a3(1);-xxw2A2a3(2) xxw2A2a3(1) 0];
xxw3A3r3=xw3*A3*r3;
chachengxxw3A3r3=[0 -xxw3A3r3(3) xxw3A3r3(2);xxw3A3r3(3) 0 -xxw3A3r3(1);-xxw3A3r3(2) xxw3A3r3(1) 0];
xxw3A3a4=xw3*A3*a4;
chachengxxw3A3a4=[0 -xxw3A3a4(3) xxw3A3a4(2);xxw3A3a4(3) 0 -xxw3A3a4(1);-xxw3A3a4(2) xxw3A3a4(1) 0];
xxw4A4r4=xw4*A4*r4;
chachengxxw4A4r4=[0 -xxw4A4r4(3) xxw4A4r4(2);xxw4A4r4(3) 0 -xxw4A4r4(1);-xxw4A4r4(2) xxw4A4r4(1) 0];
xxw4A4a5=xw4*A4*a5;
chachengxxw4A4a5=[0 -xxw4A4a5(3) xxw4A4a5(2);xxw4A4a5(3) 0 -xxw4A4a5(1);-xxw4A4a5(2) xxw4A4a5(1) 0];
xxw5A5r5=xw5*A5*r5;
chachengxxw5A5r5=[0 -xxw5A5r5(3) xxw5A5r5(2);xxw5A5r5(3) 0 -xxw5A5r5(1);-xxw5A5r5(2) xxw5A5r5(1) 0];
xxw5A5a6=xw5*A5*a6;
chachengxxw5A5a6=[0 -xxw5A5a6(3) xxw5A5a6(2);xxw5A5a6(3) 0 -xxw5A5a6(1);-xxw5A5a6(2) xxw5A5a6(1) 0];
xxw6A6r6=xw6*A6*r6;
chachengxxw6A6r6=[0 -xxw6A6r6(3) xxw6A6r6(2);xxw6A6r6(3) 0 -xxw6A6r6(1);-xxw6A6r6(2) xxw6A6r6(1) 0];

%定义dVkT ------------------------3*18矩阵
dV1T=-[chachengxxw1A1r1 [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]];
dV2T=-[chachengxxw1A1a2 chachengxxw2A2r2 [0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0]];
dV3T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3r3 [0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0]];
dV4T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4r4 [0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0]];
dV5T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4a5 chachengxxw5A5r5 [0 0 0;0 0 0;0 0 0]];
dV6T=-[chachengxxw1A1a2 chachengxxw2A2a3 chachengxxw3A3a4 chachengxxw4A4a5 chachengxxw5A5a6 chachengxxw6A6r6];
dV1=dV1T';     %18*3矩阵
dV2=dV2T';
dV3=dV3T';
dV4=dV4T';
dV5=dV5T';
dV6=dV6T';

%定义A1e3的叉乘矩阵
xA1e3=A1*e3;
chachengxA1e3=[0 -xA1e3(3) xA1e3(2);xA1e3(3) 0 -xA1e3(1);-xA1e3(2) xA1e3(1) 0];
xA2e3=A2*e3;
chachengxA2e3=[0 -xA2e3(3) xA2e3(2);xA2e3(3) 0 -xA2e3(1);-xA2e3(2) xA2e3(1) 0];
xA3e3=A3*e3;
chachengxA3e3=[0 -xA3e3(3) xA3e3(2);xA3e3(3) 0 -xA3e3(1);-xA3e3(2) xA3e3(1) 0];
xA4e3=A4*e3;
chachengxA4e3=[0 -xA4e3(3) xA4e3(2);xA4e3(3) 0 -xA4e3(1);-xA4e3(2) xA4e3(1) 0];
xA5e3=A5*e3;
chachengxA5e3=[0 -xA5e3(3) xA5e3(2);xA5e3(3) 0 -xA5e3(1);-xA5e3(2) xA5e3(1) 0];
xA6e3=A6*e3;
chachengxA6e3=[0 -xA6e3(3) xA6e3(2);xA6e3(3) 0 -xA6e3(1);-xA6e3(2) xA6e3(1) 0];

%定义dW和dWT
dW1=[dq'*W1*chachengxA1e3;[0 0 0;0 0 0;0 0 0;0 0 0;0 0 0]];  %6*3矩阵
dW2=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;[0 0 0;0 0 0;0 0 0;0 0 0]];
dW3=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;[0 0 0;0 0 0;0 0 0]];
dW4=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;[0 0 0;0 0 0]];
dW5=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;dq'*W5*chachengxA5e3;[0 0 0]];
dW6=[dq'*W1*chachengxA1e3;dq'*W2*chachengxA2e3;dq'*W3*chachengxA3e3;dq'*W4*chachengxA4e3;dq'*W5*chachengxA5e3;dq'*W6*chachengxA6e3];
dW=[dW1 dW2 dW3 dW4 dW5 dW6];  %6*18矩阵
dW1T=dW1';  %3*6矩阵
dW2T=dW2';
dW3T=dW3';
dW4T=dW4';
dW5T=dW5';
dW6T=dW6';
dWT=[dW1T;dW2T;dW3T;dW4T;dW5T;dW6T];          %18*6矩阵


%定义M矩阵
M1=m1*W*V1*V1T*WT+W1*J1*W1T;   %6*6矩阵
M2=m2*W*V2*V2T*WT+W2*J2*W2T;
M3=m3*W*V3*V3T*WT+W3*J3*W3T;
M4=m4*W*V4*V4T*WT+W4*J4*W4T;
M5=m5*W*V5*V5T*WT+W5*J5*W5T;
M6=m6*W*V6*V6T*WT+W6*J6*W6T;
M=M1+M2+M3+M4+M5+M6;   %6*6矩阵
MT=M';

%定义WkT和dq的叉乘矩阵
xW1Tdq=W1T*dq;
chachengxW1Tdq=[0 -xW1Tdq(3) xW1Tdq(2);xW1Tdq(3) 0 -xW1Tdq(1);-xW1Tdq(2) xW1Tdq(1) 0];
xW2Tdq=W2T*dq;
chachengxW2Tdq=[0 -xW2Tdq(3) xW2Tdq(2);xW2Tdq(3) 0 -xW2Tdq(1);-xW2Tdq(2) xW2Tdq(1) 0];
xW3Tdq=W3T*dq;
chachengxW3Tdq=[0 -xW3Tdq(3) xW3Tdq(2);xW3Tdq(3) 0 -xW3Tdq(1);-xW3Tdq(2) xW3Tdq(1) 0];
xW4Tdq=W4T*dq;
chachengxW4Tdq=[0 -xW4Tdq(3) xW4Tdq(2);xW4Tdq(3) 0 -xW4Tdq(1);-xW4Tdq(2) xW4Tdq(1) 0];
xW5Tdq=W5T*dq;
chachengxW5Tdq=[0 -xW5Tdq(3) xW5Tdq(2);xW5Tdq(3) 0 -xW5Tdq(1);-xW5Tdq(2) xW5Tdq(1) 0];
xW6Tdq=W6T*dq;
chachengxW6Tdq=[0 -xW6Tdq(3) xW6Tdq(2);xW6Tdq(3) 0 -xW6Tdq(1);-xW6Tdq(2) xW6Tdq(1) 0];

%定义N矩阵
N1=(m1*W*V1*(dV1T*WT+V1T*dWT)+W1*J1*dW1T+W1*chachengxW1Tdq*J1*W1T);  %6*6矩阵
N2=(m2*W*V2*(dV2T*WT+V2T*dWT)+W2*J2*dW2T+W2*chachengxW2Tdq*J2*W2T);
N3=(m3*W*V3*(dV3T*WT+V3T*dWT)+W3*J3*dW3T+W3*chachengxW3Tdq*J3*W3T);
N4=(m4*W*V4*(dV4T*WT+V4T*dWT)+W4*J4*dW4T+W4*chachengxW4Tdq*J4*W4T);
N5=(m5*W*V5*(dV5T*WT+V5T*dWT)+W5*J5*dW5T+W5*chachengxW5Tdq*J5*W5T);
N6=(m6*W*V6*(dV6T*WT+V6T*dWT)+W6*J6*dW6T+W6*chachengxW6Tdq*J6*W6T);
N=N1+N2+N3+N4+N5+N6;   %6*6矩阵

%六个力矩分量
tao(1) = u(1);   
tao(2) = u(2);   
tao(3) = u(3);   
tao(4) = u(4);   
tao(5) = u(5);   
tao(6) = u(6);   
tao=[tao(1);tao(2);tao(3);tao(4);tao(5);tao(6)];    %6*1矩阵  

f = [u(7);u(8);u(9);u(10);u(11);u(12)];
% 外部扰动
dt = 0.1*sign(t);
% 摩擦力
% f = 0.02*sign(dq);

%动力学方程
ddth=inv(M) * (tao - N * dq - dt - f); %6*1矩阵

%x1=q1    x2=q2   ....  x7=dq1  ...x12=dq6
sys(1)=x(7);   %q1
sys(2)=x(8);   %q2
sys(3)=x(9);   %q3
sys(4)=x(10);  %q4
sys(5)=x(11);  %q5
sys(6)=x(12);  %q6
sys(7)=ddth(1);    
sys(8)=ddth(2);    
sys(9)=ddth(3);     
sys(10)=ddth(4);    
sys(11)=ddth(5);    
sys(12)=ddth(6);    

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);   %q1
sys(2)=x(2);   %q2
sys(3)=x(3) ;  %q3
sys(4)=x(4) ;  %q4
sys(5)=x(5) ;  %q5
sys(6)=x(6) ;  %q6
sys(7)=x(7) ;  %dq1
sys(8)=x(8) ;  %dq2
sys(9)=x(9) ;  %dq3
sys(10)=x(10); %dq4
sys(11)=x(11); %dq5
sys(12)=x(12); %dq6

