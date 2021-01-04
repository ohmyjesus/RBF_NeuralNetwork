function [sys,x0,str,ts] = Book7242_Controller(t,x,u,flag)
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
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s
% 神经网络采用4 - 5 - 1结构  矩阵中每个值用一个W和h   M11 -- W11'*h11   M12 -- W12'*h12
node = 5;
c_M = 1 * [-1.0 -0.5 0 0.5 1;
           -1.0 -0.5 0 0.5 1];               % 高斯函数的中心点矢量 维度 IN * MID  2*5
b_M = 10 * ones(node,1);  % 高斯函数的基宽  维度node * 1  7*1   b的选择很重要 b越大 网路对输入的映射能力越大  
c_C =     [-1.0 -0.5 0 0.5 1;
          -1.0 -0.5 0 0.5 1;
          -1.0 -0.5 0 0.5 1;
          -1.0 -0.5 0 0.5 1];               % 高斯函数的中心点矢量 维度 IN * MID  4*5
b_C = 10 * ones(node,1); 
c_G = 1 * [-1.0 -0.5 0 0.5 1;
           -1.0 -0.5 0 0.5 1];               % 高斯函数的中心点矢量 维度 IN * MID  2*5
b_G = 10 * ones(node,1); 
s_new = [0; 0];
s_past = s_new;
inte_s = [0; 0];
sizes = simsizes;
sizes.NumContStates  = node*10;   %设置系统连续状态的变量 W V
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 7;   %设置系统输出的变量
sizes.NumInputs      = 12;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0 * ones(node*10,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
% qd1 = 0.5*sin(pi*t);
% qd2 = 0.5*sin(pi*t);
dqd1 =  0.5*pi*cos(pi*t);
dqd2 =  0.5*pi*cos(pi*t);
ddqd1 = -0.5*pi*pi*sin(pi*t);
ddqd2 = -0.5*pi*pi*sin(pi*t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);
ddq1 = u(7);
ddq2 = u(8);
ddq = [ddq1; ddq2];

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;
e = [e1; e2];
de = [de1; de2];
dq = [dq1; dq2];
q = [q1; q2];
dqd = [dqd1; dqd2];
ddqd = [ddqd1; ddqd2];
dde = ddqd - ddq;
% 参数的定义
kp = 100*eye(2);
ki = 100*eye(2);
kr = 0.1*eye(2);
xite = 5.0*eye(2);
gamam = 5;
gamac = 10;
gamag = 10;

s_new = xite * e + de;
% q = 3;
% p = 5;
% belta = 3;
% s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
% s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
% s_new = [s1; s2];
dqr = s_new + dq;
ddqr = xite * de + dde + ddq;

% 神经网络的输入
input1 = [q1;q2];
% ------------------------------------M矩阵的径向基函数
h_M11 = zeros(node , 1);   % 7*1
h_M12 = zeros(node , 1);
h_M21 = zeros(node , 1);
h_M22 = zeros(node , 1);
for i =1:node
    h_M11(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M12(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M21(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M22(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
end
W_M11 = x(1:node);     % 7*1
W_M12 = x(node+1: node*2);
W_M21 = x(node*2+1: node*3);
W_M22 = x(node*3+1: node*4);

% W_M权值的自适应律
dw_M11 = gamam * h_M11 * ddqr(1) * s_new(1);
dw_M12 = gamam * h_M12 * ddqr(2) * s_new(1);
dw_M21 = gamam * h_M21 * ddqr(1) * s_new(2);
dw_M22 = gamam * h_M22 * ddqr(2) * s_new(2);

for i = 1:node
    sys(i) = dw_M11(i);
    sys(i+node) = dw_M12(i);
    sys(i+node*2) = dw_M21(i);
    sys(i+node*3) = dw_M22(i);
end

% ----------------------------------------C矩阵的径向基函数
input2 = [q1;q2;dq1;dq2];
h_C11 = zeros(node , 1);   % 7*1
h_C12 = zeros(node , 1);
h_C21 = zeros(node , 1);
h_C22 = zeros(node , 1);
for i =1:node
    h_C11(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C12(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C21(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C22(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
end
W_C11 = x(node*4+1:node*5);     % 7*1
W_C12 = x(node*5+1: node*6);
W_C21 = x(node*6+1: node*7);
W_C22 = x(node*7+1: node*8);

% W_C权值的自适应律
dw_C11 = gamac * h_C11 * dqr(1) * s_new(1);
dw_C12 = gamac * h_C12 * ddqr(2) * s_new(1);
dw_C21 = gamac * h_C21 * dqr(1) * s_new(2);
dw_C22 = gamac * h_C22 * ddqr(2) * s_new(2);

for i = 1:node
    sys(i+node*4) = dw_C11(i);
    sys(i+node*5) = dw_C12(i);
    sys(i+node*6) = dw_C21(i);
    sys(i+node*7) = dw_C22(i);
end

% ----------------------------------------G矩阵的径向基函数
input3 = [q1;q2];
h_G11 = zeros(node , 1);   % 7*1
h_G12 = zeros(node , 1);
for i =1:node
    h_G11(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); % 7*1
    h_G12(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); % 7*1
end
W_G11 = x(node*8+1:node*9);     % 7*1
W_G12 = x(node*9+1: node*10);

% W_C权值的自适应律
dw_G11 = gamag * h_G11 * s_new(1);
dw_G12 = gamag * h_G12 * s_new(2);

for i = 1:node
    sys(i+node*8) = dw_G11(i);
    sys(i+node*9) = dw_G12(i);
end

function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s
% 角度跟踪指令
% qd1 = 0.5*sin(pi*t);
% qd2 = 0.5*sin(pi*t);
dqd1 =  0.5*pi*cos(pi*t);
dqd2 =  0.5*pi*cos(pi*t);
ddqd1 = -0.5*pi*pi*sin(pi*t);
ddqd2 = -0.5*pi*pi*sin(pi*t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);
ddq1 = u(7);
ddq2 = u(8);
ddq = [ddq1; ddq2];

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;
e = [e1; e2];
de = [de1; de2];
dq = [dq1; dq2];
q = [q1; q2];
dqd = [dqd1; dqd2];
ddqd = [ddqd1; ddqd2];
dde = ddqd - ddq;

% 参数的定义
kp = 100*eye(2);
ki = 100*eye(2);
xite = 5.0*eye(2);
gamam = 5;
gamac = 10;
gamag = 10;

s_new = xite * e + de;
% q = 3;
% p = 5;
% belta = 3;
% s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
% s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
% s_new = [s1; s2];
dqr = s_new + dq;
ddqr = xite * de + dde + ddq;

% ------------------------------------M矩阵的径向基函数
input1 = [q1;q2];
h_M11 = zeros(node , 1);   % 7*1
h_M12 = zeros(node , 1);
h_M21 = zeros(node , 1);
h_M22 = zeros(node , 1);
for i =1:node
    h_M11(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M12(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M21(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
    h_M22(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); % 7*1
end
W_M11 = x(1:node);     % 7*1
W_M12 = x(node*1+1: node*2);
W_M21 = x(node*2+1: node*3);
W_M22 = x(node*3+1: node*4);

% ----------------------------------------C矩阵的径向基函数
input2 = [q1;q2;dq1;dq2];
h_C11 = zeros(node , 1);   % 5*1
h_C12 = zeros(node , 1);
h_C21 = zeros(node , 1);
h_C22 = zeros(node , 1);
for i =1:node
    h_C11(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C12(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C21(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
    h_C22(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); % 7*1
end
W_C11 = x(node*4+1:node*5);     % 5*1
W_C12 = x(node*5+1: node*6);
W_C21 = x(node*6+1: node*7);
W_C22 = x(node*7+1: node*8);

% ----------------------------------------G矩阵的径向基函数
input3 = [q1;q2];
h_G11 = zeros(node , 1);   % 5*1
h_G12 = zeros(node , 1);
for i =1:node
    h_G11(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); % 7*1
    h_G12(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); % 7*1
end
W_G11 = x(node*8+1:node*9);     % 5*1
W_G12 = x(node*9+1:node*10);

% 神经网络的输出
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;
p5 = 0.87;
g = 9.8;
M = [p1+p2+2*p3*cos(q2)  p2+p3*cos(q2);
     p2+p3*cos(q2)  p2];
C = [-p3*dq2*sin(q2)  -p3*(dq1+dq2)*sin(q2);
     p3*dq1*sin(q2)  0];
G = [p4*g*cos(q1)+p5*g*cos(q1+q2);
     p5*g*cos(q1+q2)];
fx1 = [W_M11'*h_M11  W_M12'*h_M12; W_M21'*h_M21  W_M22'*h_M22];  %  2*2
fx2 = [W_C11'*h_C11  W_C12'*h_C12; W_C21'*h_C21  W_C22'*h_C22];
fx3 = [W_G11'*h_G11; W_G12'*h_G12];

M_refer = fx1;
C_refer = fx2;
G_refer = fx3;

% 名义模型控制律
taum = M_refer*ddqr + C_refer*dqr + G_refer;
% 鲁棒项
% kr = 21*eye(2);
% taur = kr*sign(s_new);

taur = [(abs(u(9))+1)*sign(s_new(1)); (abs(u(10))+1)*sign(s_new(2))];

dt = 0.001;
inte_s = inte_s + (s_past + s_new)*dt/2;

% inte_s = [u(11);u(12)];

tau = taum + kp*s_new + ki*inte_s + taur;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = norm(fx1);
sys(4) = norm(fx2);
sys(5) = norm(fx3);
sys(6) = s_new(1);
sys(7) = s_new(2);

s_past = s_new;










