function [sys,x0,str,ts] = Book6332_Controller(t,x,u,flag)
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
% 神经网络采用4 - 7 - 2结构  一个 4*7*2结构     输入[e1 e2 de1 de2] 对应输出 [tau1 tau2]
node = 7;
c = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  4*7
b = 10 * ones(node,1);  % 高斯函数的基宽  维度node * 1  7*1   b的选择很重要 b越大 网路对输入的映射能力越大  
sizes = simsizes;
sizes.NumContStates  = node*2;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 8;   %设置系统输入的变量
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
dqd1 =  cos(t);
dqd2 =  cos(t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);

e1 = q1 - qd1;      % e = qd - q
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
e = [e1; e2];
de = [de1 ; de2];

% 参数的定义
xite = 1500;
alpha = 20;
gama = 0.05;

input = [e; de];
h = zeros(node , 1);   %7*1矩阵
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (b(i)^2)); % 7*1
end

W = [x(1) x(2) x(3)  x(4)  x(5)  x(6)  x(7);
     x(8) x(9) x(10) x(11) x(12) x(13) x(14)]'; % 7*2 

x2_1 = de + alpha * e;

% 权值的自适应律
dw = -xite * x2_1 * h'; % 
% dw = dw';
for i = 1:node
    sys(i) = dw(1,i);
    sys(i+7) = dw(2,i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b node
% 角度跟踪指令
% dqd1 =  cos(t);
% dqd2 =  cos(t);
ddqd1 = -sin(t);
ddqd2 = -sin(t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);
ddq1 = u(7);
ddq2 = u(8);
dqd1 = cos(t);
dqd2 = cos(t);
ddq = [ddq1; ddq2];
ddqd = [ddqd1; ddqd2];
dqd = [dqd1; dqd2];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
e = [e1; e2];
de = [de1; de2];

% 参数的定义
xite = 1500;
alpha = 20;
gama = 0.05;
m1 = 1;
m2 = 1.5;
r1 = 1;
r2 = 0.8;
M11 = (m1 + m2)*r1^2 + m2*r2^2 + 2*m2*r1*r2*cos(q2);
M12 = m2*r2^2 + m2*r1*r2*cos(q2);
M21 = M12;
M22 = m2 * r2^2;
V12 = m2*r1*sin(q2);
G1 = (m1+m2)*r1*cos(q2) + m2*r2*cos(q1+q2);
G2 = m2*r2*cos(q1+q2);

M = [M11 M12;
     M21 M22];
V = [-V12*dq2  -V12*(dq1+dq2);
     V12*q1     0];
G = [G1; G2];
D = [10*dq1 + 30*sign(dq1); 10*dq2 + 30*sign(dq2)];

input = [e; de];
h = zeros(node , 1);   %7*1矩阵

for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (b(i)^2)); % 7*1
end
W = [x(1) x(2) x(3)  x(4)  x(5)  x(6)  x(7);
     x(8) x(9) x(10) x(11) x(12) x(13) x(14)]'; % 7*2 
 % 网络输出
fx = W' * h;

omiga = M * alpha * de + V * alpha * e;

x2_1 = de + alpha * e;

% 反馈控制律
ut = -omiga - 1/(2*gama^2) * x2_1 + fx - 1/2*x2_1;

tau = ut + M * ddqd + V * dqd + G;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx(1);
sys(4) = fx(2);













