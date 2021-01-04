function [sys,x0,str,ts] = Book4131_Controller(t,x,u,flag)
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
sizes.NumOutputs     = 2;   %设置系统输出的变量
sizes.NumInputs      = 2;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];                   % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
global W
% 神经网络采用2-5-1结构  IN = 2 MID = 5  OUT = 1
% 初始权值
W = [0 ; 0 ; 0 ; 0  ; 0]';  %MID * OUT矩阵  1*5
    
function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global W
% 神经网络采用2-5-1结构
IN = 2;
Mid = 5;
Out = 1;

Q = [500 0; 0 500];
kd = 50;
kp = 30;
gama = 1200;         %gama为正常数
b = 0.2;  % 高斯函数的基宽  维度MID * 1  1*1
c = [-2 -1 0 1 2;
    -2 -1 0 1 2];               % 高斯函数的中心点矢量 维度 IN * MID  2*5

e = u(1);
de = u(2);

some = [u(1); u(2)];
h = zeros(Mid , 1);   %5*1矩阵
for i =1:Mid
    h(i) = exp(-(norm(some - c(:,i))^2) / (2*b^2));
end
fx_refer = W * h;
gx = 133;
ddyd = -0.1 * sin(t);       %跟踪信号的二阶导数
K = [kp ;kd];
E = [e ; de];

% 控制率ut
ut = 1/(gx)*(-fx_refer + ddyd + K' * E);

sys(1) = ut;
sys(2) = fx_refer;

% 自适应律的设计
fai = [0 1; -kp -kd];
P = lyap(fai', Q);   %P为对称正定矩阵且满足Lyapunov方程
B = [0; 1];
dw = zeros(1, 5);
for i = 1:5
    dw(i) = -gama * E' * P  * B * h(i);    % 1*1 * 1*2 * 2*2 *2*1 * 1*1 
end
dt = 0.001;     % 仿真步长
W = W + dw * dt;





