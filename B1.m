function [sys,x0,str,ts] = B1(t,x,u,flag)
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
global c b kv kp
sizes = simsizes;
sizes.NumContStates  = 10;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 6;   %设置系统输出的变量
sizes.NumInputs      = 10;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 1;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0.1 * ones(1, 10);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [0 0];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
% 神经网络采用4 - 5 - 2结构
c = 1 * [-2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2];               % 高斯函数的中心点矢量 维度 IN * MID  4*5
b = 3;  % 高斯函数的基宽  维度node * 1  5*1   b的选择很重要 b越大 网路对输入的映射能力越大  
alph = 3;
kp = [alph^2 0; 0 alph^2];
kv = [2 * alph 0;0 2*alph];

function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b kv kp
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
node = 5;
Q = 50 * eye(4);
A = [zeros(2,2) eye(2,2); -kp -kv];     % 4*4
P = lyap(A' , Q);                       % 4*4
B = [0 0;0 0;1 0;0 1];

qd1 = u(1);
qd2 = u(2);  
d_qd1 = u(3);
d_qd2 = u(4);
q1 = u(5);
q2 = u(6);
dq1 = u(7);
dq2 = u(8);

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - d_qd1;
de2 = dq2 - d_qd2;

W = [x(1)  x(2)  x(3)  x(4)  x(5); x(6)  x(7)  x(8)  x(9)  x(10)]';
Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1矩阵
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
gama = 20;
dw = gama * h * Input' * P * B;
dw = dw';
for i = 1:1:5
    sys(i) = dw(1,i);
    sys(i+5) = dw(2,i);
end

% 参数的定义
% v = 13.33;
% a1 = 8.98;
% a2 = 8.75;
% g = 9.8;
% 
% M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
%      a1+a2*cos(q2)  a1];
% Q = 50 * eye(4);
% 
% 
% k1 = 0.001;
% method = 1;
% 
% % W权值的更新
% if method == 1      % 自适应方法一
%     dw = gama * h * Input' * P * B;
%     for i = 1:node*2
%         sys(i) = dw(i);
%     end
% else                % 自适应方法二
%     dw = gama * h * Input' * P * B + k1 * gama * norm(Input) * [W V];
%     for i = 1:node*2
%         sys(i) = dw(i);
%     end
% end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b kv kp
node = 5;
% 角度跟踪指令
qd1 = u(1);
qd2 = u(2);  
d_qd1 = u(3);
d_qd2 = u(4);
q1 = u(5);
q2 = u(6);
dq1 = u(7);
dq2 = u(8);
ddq1 = u(9);
ddq2 = u(10);
dd_qd1 = -0.1*pi*0.5*pi*sin(0.5*pi*t);
dd_qd2 = 0.1*pi*0.5*pi*cos(0.5*pi*t);
ddq = [ddq1; ddq2];
dd_qd = [dd_qd1; dd_qd2];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - d_qd1;
de2 = dq2 - d_qd2;
e = [e1; e2];
de = [de1; de2];

% 参数的定义
v = 13.33;
a1 = 8.98;
a2 = 8.75;
g = 9.8;

M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
     a1+a2*cos(q2)  a1];
C = [-a2*dq2*sin(q2)   -a2*(dq1 + dq2)*sin(q2);
     a2*dq1*sin(q2) 0];
G = [15*g*cos(q1)+8.75*g*cos(q1+q2);
     8.75*g*cos(q1+q2)];

dq = [dq1; dq2];

tol1 = M*(dd_qd - kv*de - kp*e) + C * dq + G;

deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;

d1 = 2;
d2 = 3;
d3 = 6;
d = d1 + d2 * norm([e1;e2]) + d3 * norm([de1; de2]);
f = inv(M) * (deltam * ddq + deltac * dq + deltag + d);

Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1矩阵
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end

W = [x(1)  x(2)  x(3)  x(4)  x(5); x(6)  x(7)  x(8)  x(9)  x(10)]';
fn = W' * h;
tol2 = -M * fn;
tol = tol1 + tol2;

sys(1) = tol(1);
sys(2) = tol(2);
sys(3) = f(1);
sys(4) = fn(1);
sys(5) = f(2);
sys(6) = fn(2);












