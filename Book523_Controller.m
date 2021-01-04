function [sys,x0,str,ts] = Book523_Controller(t,x,u,flag)
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
sizes.NumContStates  = 5;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 2;   %设置系统输出的变量
sizes.NumInputs      = 3;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = zeros(1,5);            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
global c b 
% 神经网络采用2-5-1结构
c = 0.1*[-1 -0.5  -0 0.5 1;
     -1 -0.5  -0 0.5 1];               % 高斯函数的中心点矢量 维度 IN * MID  2*5
b = 5;  % 高斯函数的基宽  维度MID * 1  1*1   b的选择很重要 b越大 网路对输入的映射能力越大  


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b gama 
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
IN = 2;
Mid = 5;
Out = 1;
yd = 0.1 * sin(t);
dyd = 0.1 * cos(t);
ddyd = -0.1 * sin(t);

c1 = 15;
gama = 0.015;
e = u(1);
de = u(2);

s = c1 * e + de;  

Input = [e; de];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %5*1矩阵
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
S = -1/gama * s  * h;

for i = 1:Mid
    sys(i) = S(i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b 
IN = 2;
Mid = 5;
Out = 1;
yd = 0.1 * sin(t);
dyd = 0.1 * cos(t);
ddyd = -0.1 * sin(t);
c1 = 15;

e = u(1);
de = u(2);
th = u(3);

s = c1 * e + de;  

Input = [e; de];

h = zeros(Mid , 1);   %13*1矩阵
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
fx = W' * h;

% 参数的定义
mc = 1;     %小车质量
m = 0.1;    %摆的质量
l = 0.5;
g_up = cos(th)/(mc+m);
g_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
g = g_up / g_down;
if t<=1.5
    xite = 1.0;
else
    xite = 0.1;
end
ut = 1 / g * (-fx + ddyd + c1 * de + xite * sign(s));
sys(1) = ut;
sys(2) = fx;












