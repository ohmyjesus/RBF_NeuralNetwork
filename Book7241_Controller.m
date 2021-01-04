function [sys,x0,str,ts] = Book7241_Controller(t,x,u,flag)
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
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
% 神经网络采用2 - 7 - 1结构  采用3个2*7*1 神经网络
node = 7;
c1 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  2*7
b1 = 20 * ones(node,1);                             % 高斯函数的基宽  维度node * 1  7*1   b的选择很重要 b越大 网路对输入的映射能力越大 
c2 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];  
b2 = 20 * ones(node,1);
c3 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];  
b3 = 20 * ones(node,1);
s_new = 0;
s_past = s_new;
inte_s = 0;
sizes = simsizes;
sizes.NumContStates  = node*3;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 4;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0 * ones(node*3,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
% qd = sin(t);
dqd =  cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);
dq = u(3);
ddq = u(4);

e = qd - q;      % e = qd - q
de = dqd - dq;
dde = ddqd - ddq;

% 参数的定义
kr = 0.1;
kp = 15;
kt = 15;
xite = 5.0;
gamam = 100;
gamac = 100;
gamag = 100;

s_new = xite * e + de;
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% 神经网络的输入
input = [q; dq];
h1 = zeros(node , 1);   %7*1矩阵
h2 = zeros(node , 1);   %7*1矩阵
h3 = zeros(node , 1);   %7*1矩阵
for i =1:node
    h1(i) = exp(-(norm(input - c1(:,i))^2) / (b1(i)^2)); % 7*1
end
for i =1:node
    h2(i) = exp(-(norm(input - c2(:,i))^2) / (b2(i)^2)); % 7*1
end
for i =1:node
    h3(i) = exp(-(norm(input - c3(:,i))^2) / (b3(i)^2)); % 7*1
end

W1 = x(1:node);     % 7*1
W2 = x(node+1: node*2);
W3 = x(node*2+1: node*3);

% 权值的自适应律
dw1 = gamam * h1 * ddqr * s_new;
dw2 = gamac * h2 * dqr * s_new;
dw3 = gamag * h3 * s_new;

for i = 1:node
    sys(i) = dw1(i);
    sys(i+7) = dw2(i);
    sys(i+14) = dw3(i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
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

e = qd - q;      % e = qd - q
de = dqd - dq;
dde = ddqd - ddq;

% 参数的定义
kr = 0.1;
kp = 15;
ki = 15;
xite = 5.0;
gamam = 100;
gamac = 100;
gamag = 100;

s_new = xite * e + de;
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% 神经网络的输入
input = [q; dq];
h1 = zeros(node , 1);   %7*1矩阵
h2 = zeros(node , 1);   %7*1矩阵
h3 = zeros(node , 1);   %7*1矩阵
for i =1:node
    h1(i) = exp(-(norm(input - c1(:,i))^2) / (b1(i)^2)); % 7*1
end
for i =1:node
    h2(i) = exp(-(norm(input - c2(:,i))^2) / (b2(i)^2)); % 7*1
end
for i =1:node
    h3(i) = exp(-(norm(input - c3(:,i))^2) / (b3(i)^2)); % 7*1
end

W1 = x(1:node);     % 7*1
W2 = x(node+1: node*2);
W3 = x(node*2+1: node*3);

% 神经网络的输出
fx1 = W1' * h1;
fx2 = W2' * h2;
fx3 = W3' * h3;

M_refer = fx1;
C_refer = fx2;
G_refer = fx3;

% 名义模型控制律
taum = M_refer*ddqr + C_refer*dqr + G_refer;
% 鲁棒项
taur = kr*sign(s_new);
dt = 0.001;
inte_s = inte_s + (s_past + s_new)*dt/2;
tau = taum + kp*s_new + ki*inte_s + taur;


sys(1) = tau;
sys(2) = fx1;
sys(3) = fx2;
sys(4) = fx3;
s_past = s_new;












