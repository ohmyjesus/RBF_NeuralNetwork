function [sys,x0,str,ts] = Fixed_time_NNcontroller(t,x,u,flag)
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
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% 神经网络采用2-7-1结构
node = 7;
c1 = 0.5*[-2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2];               % 高斯函数的中心点矢量 维度 IN * MID  2*7
b1 = 20;  % 高斯函数的基宽  维度MID * 1   b的选择很重要 b越大 网路对输入的映射能力越大  
gama1 = 10;
lg = 2;
xite = 20;   % 控制器的参数

c2 = 1.8*[-2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2];               % 高斯函数的中心点矢量 维度 IN * MID  2*7
b2 = 4;  % 高斯函数的基宽  维度MID * 1    b的选择很重要 b越大 网路对输入的映射能力越大 

gama2 = 0.01;   
belta = 2;
sizes = simsizes;
sizes.NumContStates  = node*6;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 7;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 1;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0.1*[ones(node*4,1); 2*ones(node*1,1);3*ones(node*1,1)];            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [0 0];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
yd1 = 0.5*sin(pi*t);
dyd1 = 0.5*pi*cos(pi*t);
ddyd1 = -0.5*pi*pi*sin(pi*t);

yd2 = 0.5*sin(pi*t);
dyd2 = 0.5*pi*cos(pi*t);
ddyd2 = -0.5*pi*pi*sin(pi*t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = yd1 - q1;
e2 = yd2 - q2;
de1 = dyd1 - dq1;
de2 = dyd2 - dq2;
e = [e1;e2];
de = [de1;de2];

%  参数的定义
q = 3;
p = 5;

% 滑模面
s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
s = [s1; s2];

coe1 = p/(belta*q)*de1^(p-q)^(1/q);    % 必大于0
coe2 = p/(belta*q)*de2^(p-q)^(1/q);    % 必大于0

Input = [q1;q2;dq1;dq2];
% --------------------------------------------- W权值的更新
hf1 = zeros(node , 1);   %7*1矩阵
hf2 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
end
W1 = x(1:node);         % 7*1矩阵
W2 = x(node+1:2*node);  % 7*1矩阵

fx1 = W1' * hf1;
fx2 = W2' * hf2;

fx = [fx1; fx2];           % 2*1
dw_fx1 = -gama1 * s1 * coe1 * hf1; % 7*1矩阵
dw_fx2 = -gama1 * s2 * coe2 * hf2; % 7*1矩阵

for i = 1:node
    sys(i) = dw_fx1(i);
    sys(i+node) = dw_fx2(i);
end
% ------------------------------------------------- V权值的更新
hg11 = zeros(node , 1);   %7*1矩阵
hg12 = zeros(node , 1);   %7*1矩阵
hg21 = zeros(node , 1);   %7*1矩阵
hg22 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hg11(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg12(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg21(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg22(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
end
V11 = x(node*2+1:node*3);
V12 = x(node*3+1:node*4);
V21 = x(node*4+1:node*5);
V22 = x(node*5+1:node*6);

gx11 = V11' * hg11;
gx12 = V12' * hg12;
gx21 = V21' * hg21;
gx22 = V22' * hg22;

gx = [gx11  gx12; gx21 gx22];
% 控制器的设计
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;

M = [p1+p2+2*p3*cos(q2)  p2+p3*cos(q2);
     p2+p3*cos(q2)  p2];
% 控制器的设计
g = inv(M);
temp1 = -belta*q/p* (abs(de1)^(2-p/q)*sign(de1)) + fx(1) - (lg+xite)*sign(s1) - ddyd1;
temp2 = -belta*q/p* (abs(de2)^(2-p/q)*sign(de2)) + fx(2) - (lg+xite)*sign(s2) - ddyd2;
temp = [temp1; temp2];
tau = -inv(gx) * temp;

dw_g11 = -gama2 * s1 * coe1 *  hg11 * tau(1);
dw_g12 = -gama2 * s1 * coe1 *  hg12 * tau(1);

dw_g21 = -gama2 * s2 * coe2 *  hg21 * tau(2);
dw_g22 = -gama2 * s2 * coe2 *  hg22 * tau(2);

for i = 1 : node
    sys(i+node*2) = dw_g11(i);
    sys(i+node*3) = dw_g12(i);
    sys(i+node*4) = dw_g21(i);
    sys(i+node*5) = dw_g22(i);
end

function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% 角度跟踪指令
yd1 = 0.5*sin(pi*t);
dyd1 = 0.5*pi*cos(pi*t);
ddyd1 = -0.5*pi*pi*sin(pi*t);

yd2 = 0.5*sin(pi*t);
dyd2 = 0.5*pi*cos(pi*t);
ddyd2 = -0.5*pi*pi*sin(pi*t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = yd1 - q1;
e2 = yd2 - q2;
de1 = dyd1 - dq1;
de2 = dyd2 - dq2;
e = [e1;e2];
de = [de1;de2];
ddyd = [ddyd1; ddyd2];

%  参数的定义
q = 3;
p = 5;

% 滑模面
s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
s = [s1; s2];

Input = [q1;q2;dq1;dq2];
% ------------------------------------------- W权值
hf1 = zeros(node , 1);   %7*1矩阵
hf2 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
end
W1 = x(1:node);         % 7*1矩阵
W2 = x(node+1:2*node);  % 7*1矩阵

coe1 = p/(belta*q)*de1^(p-q)^(1/q);    % 必大于0
coe2 = p/(belta*q)*de2^(p-q)^(1/q);    % 必大于0

fx1 = W1' * hf1;
fx2 = W2' * hf2;

fx = [fx1; fx2];           % 2*1

% ------------------------------------------- V权值
hg11 = zeros(node , 1);   %7*1矩阵
hg12 = zeros(node , 1);   %7*1矩阵
hg21 = zeros(node , 1);   %7*1矩阵
hg22 = zeros(node , 1);   %7*1矩阵
for i =1:node
    hg11(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg12(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg21(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg22(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
end
V11 = x(node*2+1:node*3);
V12 = x(node*3+1:node*4);
V21 = x(node*4+1:node*5);
V22 = x(node*5+1:node*6);

gx11 = V11' * hg11;
gx12 = V12' * hg12;
gx21 = V21' * hg21;
gx22 = V22' * hg22;

gx = [gx11  gx12; gx21 gx22];
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;

M = [p1+p2+2*p3*cos(q2)  p2+p3*cos(q2);
     p2+p3*cos(q2)  p2];
% 控制器的设计
g = inv(M);

temp1 = -belta*q/p* (abs(de1)^(2-p/q)*sign(de1)) + fx(1) - (lg+xite)*sign(s1) - ddyd1;
temp2 = -belta*q/p* (abs(de2)^(2-p/q)*sign(de2)) + fx(2) - (lg+xite)*sign(s2) - ddyd2;
temp = [temp1; temp2];
tau = - inv(gx) * temp;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx1;
sys(4) = fx2;














