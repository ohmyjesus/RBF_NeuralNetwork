function [sys,x0,str,ts] = Book523_Plant(t,x,u,flag)
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
sizes.NumOutputs     = 2;   %设置系统输出的变量
sizes.NumInputs      = 1;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [pi/60 0];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
% 倒立摆状态方程
ut = u(1);     
% f = u(2);
th = x(1);      % 摆角
dth = x(2);     % 摆速

% 参数的定义
g = 9.8;
mc = 1;     %小车质量
m = 0.1;    %摆的质量
l = 0.5;
f_up = g*sin(th) - m*l*dth^2*cos(th)*sin(th)/(mc+m);
f_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
f = f_up / f_down;
g_up = cos(th)/(mc+m);
g_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
g = g_up / g_down;

some = f + g * ut;

sys(1) = x(2);   
sys(2) = some;   
 

function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
sys(1) = x(1);   %x1
sys(2) = x(2);   %x2




