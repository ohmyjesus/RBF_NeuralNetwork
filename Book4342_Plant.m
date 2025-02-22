function [sys,x0,str,ts] = Book4342_Plant(t,x,u,flag)
% 以下程序是 基于RBF神经网络的直接鲁棒自适应控制
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
x0  = [0 0];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
ut = u(1);      
th = x(1);      % 摆角
dth = x(2);     % 摆速
fx_up = 0.5*sin(th)*(1+0.5*cos(th))*dth^2 - 10*sin(th)*(1+cos(th));
fx_down = 0.25*(2+cos(th))^2;
fx = fx_up / fx_down;
gx = 1 / (0.25*(2+cos(th))^2);
d1 = cos(3*t);
dt = 0.1 * d1 * cos(th);

sys(1) = x(2);   
sys(2) = fx + gx * ut + dt;   
 

function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
sys(1) = x(1);   %x1
sys(2) = x(2);   %x2




