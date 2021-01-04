function [sys,x0,str,ts] = ctrl1_zuo(t,x,u,flag)
switch flag
      case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
      case {1,2,4,9}
    sys=[];
      case 3
    sys=mdlOutputs(t,x,u);
      otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;  
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)
% 跟踪轨迹
% dr = 0.5*pi*cos(0.5*pi*t);
% ddr = -0.25*pi*pi*sin(0.5*pi*t);
r = u(1);
dr = u(2);
ddr = u(3);
z1 = u(4);
z2 = u(5);

% 参数的定义
belta = 1;
q = 3;
p = 5;

d = sin(10*z1)+cos(z2); %干扰
g = 9.8;
mc = 1;
m = 0.1;
l = 0.5;
lg = 2;
xite = 10;

M = mc + m;
% e = z1 - r;
% de = z2 - dr;
e = r - z1;
de = dr - z2;
v1 = g*sin(z1)- m*l*(z2)^2*cos(z1)*sin(z1)/M;
v2 = l*(4/3-(m*cos(z1)*cos(z1))/M);
f = v1/v2;
v3 = cos(z1)/M;
v4 = v2;
g = v3/v4;

% 滑模面
s = -e + 1/belta*abs(-de)^(p/q)*sign(-de);

% 控制器的设计
ut = -1/g*(belta*q/p* (abs(-de)^(2-p/q)*sign(-de)) + f+(lg+xite)*sign(s));

temp = (-de)^(p-q)^(1/q);
sys(1) = ut;
sys(2) = temp;


