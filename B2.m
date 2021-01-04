function [sys,x0,str,ts] = B2(t,x,u,flag)
switch flag
  case 0 %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1 %连续状态计算
    sys=mdlDerivatives(t,x,u);
  case {2,4,9} %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];
  case 3 %输出信号计算
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 4;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 2;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [0.6 0.3 0.5 0.5];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
persistent ddx1 ddx2
if t == 0
    ddx1 = 0;
    ddx2 = 0;
end
% 角度跟踪指令
qd1 = 1+0.2*sin(0.5*pi*t);
qd2 = 1-0.2*cos(0.5*pi*t);
dqd1 = 0.1*pi*cos(0.5*pi*t);
dqd2 = 0.1*pi*sin(0.5*pi*t);

e1 = x(1) - qd1;
e2 = x(3) - qd2;
de1 = x(2) - dqd1;
de2 = x(4) - dqd2;

q1 = x(1);
q2 = x(3);
dq1 = x(2);
dq2 = x(4);

% 参数的定义
v = 13.33;
a1 = 8.98;
a2 = 8.75;
g = 9.8;

M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
     a1+a2*cos(q2)  a1];
C = [-a2*dq2*sin(q2)   -a2*(dq1 + dq2)*sin(q2);
     a2*dq1*sin(q2) 0];
G = [15*g*cos(q1)+8.75*g*cos(q1+q2);
     8.75*g*cos(q1+q2)];
deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;

d1 = 2;
d2 = 3;
d3 = 6;
d = d1 + d2 * norm([e1;e2]) + d3 * norm([de1; de2]);
tol(1) = u(1);    %力矩1
tol(2) = u(2);    %力矩2

dq = [x(2); x(4)];
ddq = [ddx1; ddx2];
f = inv(M) * (deltam * ddq + deltac * dq + deltag + d);

ddx = inv(M) * (tol' - C* dq - G) + f;

sys(1) = x(2);   
sys(2) = ddx(1);
sys(3) = x(4);
sys(4) = ddx(2);
ddx1 = ddx(1);
ddx2 = ddx(2);
 

function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
sys(1) = x(1);   %q1
sys(2) = x(2);   %dq1
sys(3) = x(3);   %q2
sys(4) = x(4);   %dq2



