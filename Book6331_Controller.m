function [sys,x0,str,ts] = Book6331_Controller(t,x,u,flag)
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
global c b node
% 神经网络采用2 - 5 - 1结构  
node = 5;
c = 1 * [-1.0 -0.5 0 0.5 1 ;
         -1.0 -0.5 0 0.5 1];               % 高斯函数的中心点矢量 维度 IN * MID  2*5
b = 50 * ones(node,1);  % 高斯函数的基宽  维度node * 1  7*1   b的选择很重要 b越大 网路对输入的映射能力越大  
sizes = simsizes;
sizes.NumContStates  = node;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 2;   %设置系统输出的变量
sizes.NumInputs      = 4;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0 * ones(node,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b node
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
% qd = sin(t);
dqd =  cos(t);

qd = u(1);
q = u(2);
dq = u(3);

e = q - qd;      % e = q - qd
de = dq - dqd;

% 参数的定义
xite = 1000;
alpha = 200;
gama = 0.1;

input = [e; de];
h = zeros(node , 1);   %7*1矩阵
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (2*b(i)^2)); % 7*1
end

W = x(1:node);     % 5*1 
% 权值的自适应律
x_2 = de + alpha * e;
dw = - xite * x_2 * h';
for i = 1:node
    sys(i) = dw(i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b node
% 角度跟踪指令
% dqd1 =  0.1*cos(t);
% dqd2 =  0.1*cos(t);
% qd = sin(t);
dqd =  cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);
dq = u(3);
ddq = u(4);

e = q - qd;      % e = q - qd
de = dq - dqd;

% 参数的定义
M = 1;
xite = 1000;
alpha = 200;
gama = 0.1;

input = [e; de];
h = zeros(node , 1);   %5*1矩阵
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (2*b(i)^2)); % 5*1
end
W = x(1:node);     % 5*1 

% 神经网络的输出
fx = W' * h;
V = 0;
omiga = M * alpha * de + V * alpha * e;
x_2 = de + alpha * e;
ut = -omiga - 1/(2*gama*gama)*x_2 + W' * h - 1/2*x_2;
G = 0;
tau = ut + M*ddqd + V * dqd + G;

sys(1) = tau;
sys(2) = fx;













