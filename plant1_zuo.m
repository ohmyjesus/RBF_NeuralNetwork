function [sys,x0,str,ts] = plant1_zuo(t,x,u,flag)
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
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;   
sys = simsizes(sizes);
x0  = [1 0.5];
str = [];
ts  = [];

function sys=mdlDerivatives(t,x,u)
% 收敛控制
d = sin(10*x(1))+cos(x(2)); %干扰
ut = u(1);
z1 = x(1);
z2 = x(2);

% 参数的定义
g = 9.8;
mc = 1;
m = 0.1;
l = 0.5;

M = mc + m;
v1 = g*sin(z1)- m*l*(z2)^2*cos(z1)*sin(z1)/M;
v2 = l*(4/3-(m*cos(z1)*cos(z1))/M);
f = v1/v2;
v3 = cos(z1)/M;
v4 = v2;
g = v3/v4;

sys(1) = x(2);
sys(2) = f + g*ut + d;

function sys=mdlOutputs(t,x,u)

sys(1) = x(1);
sys(2) = x(2);











