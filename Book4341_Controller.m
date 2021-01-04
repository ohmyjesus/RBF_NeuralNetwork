function [sys,x0,str,ts] = Book4341_Controller(t,x,u,flag)
% 以下程序是 基于RBF神经网络的直接鲁棒自适应控制
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
sizes.NumInputs      = 4;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];                   % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
global W 
% 神经网络采用5-9-1结构  IN = 5 MID = 9  OUT = 1
% 初始权值
W = [0 0 0 0 0 0 0 0 0]' ;  %MID * OUT矩阵  9*1

    
function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global W 
% 神经网络采用5-9-1结构
b = 20;  % 高斯函数的基宽  维度MID * 1  1*1   b的选择很重要 b越大 网路对输入的映射能力越大  
c = [-2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2];               % 高斯函数的中心点矢量 维度 IN * MID  5*9
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
IN = 5;
Mid = 9;
Out = 1;

lambda = 5;
ita = 500 * eye(9);
xite = 0.005;
If = 0.25;

e = -u(1);       % e = x - xd; 实际-期望
de = -u(2);
x_1 = u(3);
x_2 = u(4);
s = lambda * e + de;
s_if = s/If;

yd = sin(t);
dyd = cos(t);
ddyd = -sin(t);

v = -ddyd + lambda * de;

Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %9*1矩阵
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
% 控制率ut
belta = 150;
ut = 1/belta * W' * h;
sys(1) = ut;

% W权值的更新
dw = -ita * (h * s + xite * W); % 9*9 * (9*1  +  9*1)
dt = 0.001;     % 仿真步长
W = W + dw * dt;    % W的自适应律







