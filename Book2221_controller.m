function [sys,x0,str,ts] = Book2221_controller(t,x,u,flag)
% RBF神经网络自适应控制刘金琨例题2.2.2.1仿真
% 基于梯度下降法的RBF神经网络逼近
switch flag
  case 0 %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case {1,2,4,9} %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];
  case 3 %输出信号计算
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
x0  = [];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
% 权值初值的选择
% 神经网络PID控制器 2-5-1结构
global  W_new W_past C  B 
C = [-1  -0.5  0  0.5  1;  
        -10  -5  0  5  10];   %2*5  中心矢量
B = [1.5  1.5  1.5  1.5  1.5];  %1*5  基宽度参数
W_new = rand(1,5);
W_past = W_new;


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global W_new W_past C  B 
alpha = 0.05; %惯性系数
xite = 0.5;   %学习效率
u_in = u(1);
y_out = u(2);  
some = [u_in; y_out];
h = zeros(5,1);
for j = 1:5
    h(j) = exp(-norm(some - C(:,j))^2/(2 * B(j)^2));  %6*1矩阵  径向基函数
end
% RBF的网络输出ym
ym = W_new * h;

% 权值的调整 更新值
deltaW_new = zeros(1,5);
for i = 1:5
    deltaW_new(i) = xite * (y_out - ym) * h(i);
end

for i = 1:5
    W_new(i) = W_new(i) + deltaW_new(i) + alpha*(W_new(i) - W_past(i));
end

sys(1) = ym;

W_past = W_new;




