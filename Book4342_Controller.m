function [sys,x0,str,ts] = Book4342_Controller(t,x,u,flag)
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
global c b node If lamda W
W = [0 0 0 0 0 0 0 0 0 0 0 0 0]' ;  %MID * OUT 矩阵  13*1
node = 13;
If = 0.25;
lamda = 5;
sizes = simsizes;
sizes.NumContStates  = node;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 1;   %设置系统输出的变量
sizes.NumInputs      = 3;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = zeros(1,13);            % 系统初始状态变量
% 神经网络采用5-9-1结构
c = [-6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6];               % 高斯函数的中心点矢量 维度 IN * MID  5*13
b = 5;  % 高斯函数的基宽  维度MID * 1  1*1   b的选择很重要 b越大 网路对输入的映射能力越大  
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c b node If lamda W
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
IN = 5;
Mid = 13;
Out = 1;
yd = pi/6 * sin(t);
dyd = pi/6 * cos(t);
ddyd = -pi/6 * sin(t);

% e = -u(1);       % e = x - xd; 实际-期望
% de = -u(2);
% x_1 = u(3);
% x_2 = u(4);
x1 = u(2);
x2 = u(3);
e = x1 - yd;
de = x2 - dyd;
s = lamda * e + de;  
s_if = s/If;

v = -ddyd + lamda * de;
Input = [x1; x2; s; s_if ; v];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %13*1矩阵
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end

rou = 0.005;
Gama = 15 * eye(13);
W = [x(1); x(2); x(3); x(4); x(5); x(6); x(7); x(8); x(9); x(10); x(11); x(12); x(13)];
S = -Gama * (h*s + rou*W);

for i = 1:node
    sys(i) = S(i);
end


function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c b node If lamda W
yd = pi/6 * sin(t);
dyd = pi/6 * cos(t);
ddyd = -pi/6 * sin(t);
x_1 = u(2);
x_2 = u(3);
e = x_1 - yd;   % e = x - xd; 实际-期望
de = x_2 - dyd;
s = lamda * e + de;  
s_if = s/If;

v = -ddyd + lamda * de;
Input = [x_1; x_2; s; s_if ; v];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(node , 1);   %13*1矩阵
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5); x(6); x(7); x(8); x(9); x(10); x(11); x(12); x(13)];
belta = 1;
ut = 1/belta * W' * h;
sys(1) = ut;












