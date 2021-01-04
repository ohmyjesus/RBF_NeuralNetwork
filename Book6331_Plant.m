function [sys,x0,str,ts] = Book6331_Plant(t,x,u,flag)
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
sizes.NumContStates  = 2;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 3;   %设置系统输出的变量
sizes.NumInputs      = 1;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [0 0];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
tau = u(1);    %力矩1

q = x(1);      % 关节角一
dq = x(2);     % 关节角速度一

% 参数的定义
M = 1.0;
d = 150 * sign(dq) + 10 * dq;

ddq = inv(M) * (tau - d);

sys(1) = x(2);   
sys(2) = ddq;


function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
tau = u(1);    %力矩1

q = x(1);      % 关节角一
dq = x(2);     % 关节角速度一

% 参数的定义
M = 1.0;
d = 150 * sign(dq) + 10 * dq;

ddq = inv(M) * (tau - d);
sys(1) = x(1);   %q1
sys(2) = x(2);   %dq1
sys(3) = d;




