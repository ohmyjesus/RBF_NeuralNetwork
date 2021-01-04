function [sys,x0,str,ts] = Book6242_Controller(t,x,u,flag)
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
% 神经网络采用4 - 7 - 2结构
node = 7;
c = 1 * [-1.5 -1  -0.5 0 0.5 1 1.5;
         -1.5 -1  -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  2*7
b = 10 * ones(node,1);  % 高斯函数的基宽  维度node * 1  5*1   b的选择很重要 b越大 网路对输入的映射能力越大  
sizes = simsizes;
sizes.NumContStates  = node*2;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 6;   %设置系统输出的变量
sizes.NumInputs      = 6;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0 * ones(node*2,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b node
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
qd1 = 0.1*sin(t);
qd2 = 0.1*sin(t);
dqd1 = 0.1*cos(t);
dqd2 = 0.1*cos(t);
  
q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;

% 参数的定义
kv = [10 0; 0 10];
ita = [15 0;0 15];
xite1 = 5;
xite2 = 5;

% 滑模函数
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;
s = [s1;s2];

Input1 = [e1;de1];
hw = zeros(node, 1);   %7*1矩阵
for j = 1:node
    hw(j) = exp(-(norm(Input1 - c(:,j))^2) / (b(j)^2));
end

Input2 = [e2;de2];
hv = zeros(node, 1);   %7*1矩阵
for j = 1:node
    hv(j) = exp(-(norm(Input2 - c(:,j))^2) / (b(j)^2));
end

W = x(1:node);     % node*1
V = x(node+1:2*node);

% W权值的更新
dw1 = ita(1)*hw*s(1);
dw2 = ita(4)*hv*s(2);
for i = 1:node
    sys(i) = dw1(i);
    sys(i+node) = dw2(i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b node
% 角度跟踪指令
qd1 = 0.1*sin(t);
qd2 = 0.1*sin(t);
dqd1 = 0.1*cos(t);
dqd2 = 0.1*cos(t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);
K1 = u(5);
K2 = u(6);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;

% 参数的定义
kv = [20 0; 0 20];
ita = [15 0;0 15];
xite1 = 5;
xite2 = 5;
epc = 2;
bd = 2.1;

% 滑模函数
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;

d1 = norm(s1)/sqrt(xite1^2+1);
d2 = norm(s2)/sqrt(xite2^2+1);

% 滑模函数
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;
s = [s1;s2];

Input1 = [e1;de1];
hw = zeros(node, 1);   %7*1矩阵
for j = 1:node
    hw(j) = exp(-(norm(Input1 - c(:,j))^2) / (b(j)^2));
end

Input2 = [e2;de2];
hv = zeros(node, 1);   %7*1矩阵
for j = 1:node
    hv(j) = exp(-(norm(Input2 - c(:,j))^2) / (b(j)^2));
end

W = x(1:node);     % node*1
V = x(node+1:2*node);

% 神经网络的输出
fx1 = W' * hw;
fx2 = V' * hv;
v = -(epc+bd)*sign(s);

temp1 = (abs(K1)+1)*sign(s1);
temp2 = (abs(K2)+1)*sign(s2);
v_new = -[temp1; temp2];

tau = [fx1;fx2]+kv*s-v;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx1;
sys(4) = fx2;
sys(5) = s1;
sys(6) = s2;











