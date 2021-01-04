function [sys,x0,str,ts] = Book331_Controller(t,x,u,flag)
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

function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 0;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 1;   %设置系统输出的变量
sizes.NumInputs      = 2;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];                   % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
global W_past W_new V_new V_past c b ut_new ut_past
% 神经网络采用1-6-1结构
W_new = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5];  %6*1矩阵
W_past = W_new;
V_new = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5];  %6*1矩阵
V_past = V_new;
c = [0.5  0.5  0.5  0.5  0.5  0.5];
b = [5;5;5;5;5;5];
ut_new = 0;
ut_past = ut_new;
    
function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
global W_past W_new V_new V_past c b ut_new ut_past
% 神经网络采用1-6-1结构
xitew = 0.15;   %学习速率1
xitev = 0.5;    %学习速率2
alfa = 0.05;   %动量因子
IN = 1;
Mid = 6;
Out = 1;
yd = u(1);
yk = u(2);

some = [u(1); u(2)];
h = zeros(1,6);   %1*6矩阵
for i =1:Mid
    h(i) = exp(-(norm(yk - c(:,i))^2) / (2*b(i)^2));
end
ut1 = h * W_new;
ut2 = h * V_new;

ut_new = -(ut1/ut2) + (yd/ut2);
sys(1) = ut_new;

% 权值的修正
deltaw = zeros(6,1);
for i = 1:6
    deltaw(i) = xitew * (yk - yd) * h(i);
end
W_new = W_new + deltaw + alfa*(W_new - W_past);

deltav = zeros(6,1);
for i = 1:6
    deltav(i) = xitev * (yk - yd) * h(i) * ut_new;
end
V_new = V_new + deltav + alfa*(V_new - V_past);

W_past = W_new;
V_past = V_new;
ut_past = ut_new;






