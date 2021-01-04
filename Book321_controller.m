function [sys,x0,str,ts] = Book321_controller(t,x,u,flag)
% RBF神经网络自适应控制刘金琨例题3.2.1仿真
% 基于RBF神经网络的模型参考自适应控制
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
sizes.NumInputs      = 3;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1 = -1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
% 权值初值的选择
% 神经网络PID控制器 2-7-1结构
global  W_new W_past C_new C_past  B_new B_past yk_new yk_past ut_new ut_past some
C_new = [-3  -2  -1  0  1  2  3;
        -3  -2  -1  0  1  2  3;
        -3  -2  -1  0  1  2  3];   %3*7  中心矢量
C_past = C_new;
B_new = [2  2  2  2  2  2  2];  %1*7  基宽度参数
B_past = B_new;
% W_new = rand(1,7);  %权值取0-1的随机值
W_new = [-0.0316  -0.0421 -0.0318 0.0068 0.0454 -0.0381 -0.0381];
W_past = W_new;
yk_new = 0;
yk_past = yk_new;
ut_new = 0;
ut_past = ut_new;
some = [0 0 0]';

function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global W_new W_past C_new C_past  B_new B_past  yk_new yk_past ut_new ut_past some
% if t>0
% 神经网络PID控制器 2-6-1结构
alpha = 0.05; %惯性系数
xite = 0.35;   %学习效率
IN = 3;
Mid = 7;
Out = 1;

yd = u(1);
ec = u(2);  
yk_new = u(3);

h = zeros(Mid, 1);
for j = 1:Mid
    h(j) = exp(-(norm(some - C_new(:,j))^2/(2* B_new(j)^2)));  %7*1矩阵  径向基函数
end
% RBF的网络输出ym
ut_new = W_new * h;
dyu = sign((yk_new - yk_past)/(ut_new - ut_past));

% 权值的调整 更新值
deltaW = zeros(1,Mid);
for i = 1:Mid
    deltaW(i) = xite *  ec * dyu * h(i);
end
W_new = W_new + deltaW + alpha*(W_new - W_past);

% 基宽带参数b的修正
% deltab = zeros(1,Mid);
% for i = 1:Mid
%     deltab(i) = xite * dyu * W_new(i) * h(i) * (norm(some - C_new(:,i))^2 / B_new(i)^3);
%     B_new(i) = B_new(i) + deltab(i) + alpha*(B_new(i) - B_past(i));
% end

% 中心矢量c的修正
% deltac = zeros(IN, Mid);
% for i = 1:IN
%     for j = 1:Mid 
%         deltac(i,j) = xite * dyu * W_new(j) * h(j) * ((some(i) - C_new(i,j)) / B_new(j)^2);
%         C_new(i,j) = C_new(i,j) + deltac(i,j) + alpha*(C_new(i,j) - C_past(i,j));
%     end
% end

sys(1) = ut_new;
some = [u(1); u(2); u(3)];
W_past = W_new;
B_past = B_new;
C_past = C_new;
ut_past = ut_new;
yk_past = yk_new;
% else
%     sys(1) = ut_new;
% end



