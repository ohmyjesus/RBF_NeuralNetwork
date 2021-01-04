function [sys,x0,str,ts] = Book6332_Plant(t,x,u,flag)
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
sizes.NumOutputs     = 6;   %设置系统输出的变量
sizes.NumInputs      = 2;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [0 0 0 0];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
% 角度跟踪指令
% qd1 = sin(t);
% qd2 = sin(t);
% dqd1 = cos(t);
% dqd2 = cos(t);
tau1 = u(1);    %力矩1
tau2 = u(2);    %力矩2

q1 = x(1);      % 关节角一
q2 = x(2);      % 关节角二
dq1 = x(3);     % 关节角速度一
dq2 = x(4);     % 关节角速度一
q = [q1; q2];
dq = [dq1; dq2];

% 参数的定义
m1 = 1;
m2 = 1.5;
r1 = 1;
r2 = 0.8;
M11 = (m1 + m2)*r1^2 + m2*r2^2 + 2*m2*r1*r2*cos(q2);
M12 = m2*r2^2 + m2*r1*r2*cos(q2);
M21 = M12;
M22 = m2 * r2^2;
V12 = m2*r1*sin(q2);
G1 = (m1+m2)*r1*cos(q2) + m2*r2*cos(q1+q2);
G2 = m2*r2*cos(q1+q2);

M = [M11 M12;
     M21 M22];
V = [-V12*dq2  -V12*(dq1+dq2);
     V12*q1     0];
G = [G1; G2];
D = [10*dq1 + 30*sign(dq1); 10*dq2 + 30*sign(dq2)];

tau = [tau1; tau2];

ddq = inv(M) * (tau - D - G - V*dq);

sys(1) = x(3);   
sys(2) = x(4);
sys(3) = ddq(1);
sys(4) = ddq(2);


function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
tau1 = u(1);    %力矩1
tau2 = u(2);    %力矩2

q1 = x(1);      % 关节角一
q2 = x(2);      % 关节角二
dq1 = x(3);     % 关节角速度一
dq2 = x(4);     % 关节角速度一
q = [q1; q2];
dq = [dq1; dq2];

% 参数的定义
m1 = 1;
m2 = 1.5;
r1 = 1;
r2 = 0.8;
M11 = (m1 + m2)*r1^2 + m2*r2^2 + 2*m2*r1*r2*cos(q2);
M12 = m2*r2^2 + m2*r1*r2*cos(q2);
M21 = M12;
M22 = m2 * r2^2;
V12 = m2*r1*sin(q2);
G1 = (m1+m2)*r1*cos(q2) + m2*r2*cos(q1+q2);
G2 = m2*r2*cos(q1+q2);

M = [M11 M12;
     M21 M22];
V = [-V12*dq2  -V12*(dq1+dq2);
     V12*q1     0];
G = [G1; G2];
D = [10*dq1 + 30*sign(dq1); 10*dq2 + 30*sign(dq2)];

sys(1) = x(1);   %q1
sys(2) = x(2);   %q2
sys(3) = x(3);   %dq1
sys(4) = x(4);   %dq2
sys(5) = D(1);
sys(6) = D(2);


