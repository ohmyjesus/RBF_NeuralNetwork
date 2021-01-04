function [sys,x0,str,ts] = Book2222_controller(t,x,u,flag)
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
global  W_new W_past C_new C_past  B_new B_past 
C_new = [-1  -0.5  0  0.5  1;  
        -10  -5  0  5  10];   %2*5  中心矢量
C_past = C_new;
B_new = [3  3  3  3  3];  %1*5  基宽度参数
B_past = B_new;
W_new = rand(1,5);  %权值取0-1的随机值
W_past = W_new;


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global W_new W_past C_new C_past  B_new B_past 
alpha = 0.05; %惯性系数
xite = 0.15;   %学习效率
u_in = u(1);
y_out = u(2);  
some = [u_in; y_out];
h = zeros(5,1);
for j = 1:5
    h(j) = exp(-(norm(some - C_new(:,j))^2/(2* B_new(j)^2)));  %5*1矩阵  径向基函数
end
% RBF的网络输出ym
ym = W_new * h;

% 权值的调整 更新值
deltaW = zeros(1,5);
for i = 1:5
    deltaW(i) = xite * (y_out - ym) * h(i);
end
for i = 1:5
    W_new(i) = W_new(i) + deltaW(i) + alpha*(W_new(i) - W_past(i));
end

% 基宽带参数b的修正
deltab = zeros(1,5);
for i = 1:5
    deltab(i) = xite * (y_out - ym) * W_new(i) * h(i) * (norm(some - C_new(:,i))^2 / B_new(i)^3);
    B_new(i) = B_new(i) + deltab(i) + alpha*(B_new(i) - B_past(i));
end

% 中心矢量c的修正
deltac = zeros(2,5);
for j = 1:2
    for i = 1:5 
        deltac(j,i) = xite * (y_out - ym) * W_new(i) * h(i) * ((some(j) - C_new(j,i)) / B_new(i)^2);
        C_new(j,i) = C_new(j,i) + deltac(j,i) + alpha*(C_new(j,i) - C_past(j,i));
    end
end

sys(1) = ym;

W_past = W_new;
B_past = B_new;
C_past = C_new;



