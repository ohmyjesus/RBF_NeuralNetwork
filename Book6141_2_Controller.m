function [sys,x0,str,ts] = Book6141_2_Controller(t,x,u,flag)
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
global c b node
% 神经网络采用2-19-1结构
node = 19;
c = 0.5 * [-4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5 3 3.5 4 4.5;
           -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5 3 3.5 4 4.5];               % 高斯函数的中心点矢量 维度 IN * MID  2*19
b = 2 * ones(19,1);  % 高斯函数的基宽  维度node * 1  19*1   b的选择很重要 b越大 网路对输入的映射能力越大  
sizes = simsizes;
sizes.NumContStates  = node;   %设置系统连续状态的变量 W
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 3;   %设置系统输出的变量
sizes.NumInputs      = 3;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0.1 * ones(node,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b node
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
IN = 2;
Out = 1;
qd = sin(t);
dqd = cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);      % e = q - qd
dq = u(3);
e = q - qd;
de = dq - dqd;

% 参数的定义
M = 10;
gama = 1200;
alph = 3;
kp = alph^2;
kv = 2 * alph;
Q = [50 0; 0 50];
A = [0 1; -kp -kv];
P = lyap(A' , Q);
B = [0; 1/M];
k1 = 0.001;

Input = [e; de];
h = zeros(node , 1);   %19*1矩阵
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = [x(1:19)]';     %node*1  19*1

method = 1;
if method == 1      % 自适应方法一
    dw = gama * h * Input' * P * B;
    for i = 1:node
    sys(i) = dw(i);
    end
else                % 自适应方法二
    dw = gama * h * Input' * P * B + k1 * gama * norm(Input) * W;
    for i = 1:node
    sys(i) = dw(i);
    end
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b node
IN = 2;
Out = 1;
qd = sin(t);
dqd = cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);      % e = q - qd
dq = u(3);
e = q - qd;
de = dq - dqd;

% 参数的定义
M = 10;
gama = 1200;
alph = 3;
kp = alph^2;
kv = 2 * alph;
Q = [50 0; 0 50];
A = [0 1; -kp -kv];
P = lyap(A' , Q);
B = [0; 1/M];
k1 = 0.001;

Input = [e; de];
h = zeros(node , 1);   %5*1矩阵
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = [x(1:19)];     %node*1  19*1
% 神经网络的输出
fx = W' * h;
d = -15 * dq - 30 *sign(dq);

some = 1;
if some == 1
    ut = M * (ddqd - kv*de - kp*e) - fx;
elseif some == 2
    ut = M * (ddqd - kv*de - kp*e) - d;
else
    ut = M * (ddqd - kv*de - kp*e);
end
sys(1) = ut;
sys(2) = fx;
sys(3) = d;












