function [sys,x0,str,ts] = CRP_Controller(t,x,u,flag)
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
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s_past inte_s_new gamam gamac gamag kp ki kr xite
% 神经网络采用4 - 5 - 1结构  矩阵中每个值用一个W和h   M11 -- W11'*h11   M12 -- W12'*h12
node = 7;
c_M = 2 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  6*7
b_M = 10 * ones(node,1);  % 高斯函数的基宽  维度node * 1  7*1   b的选择很重要 b越大 网路对输入的映射能力越大  
c_C =  2 *  [-1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5;
          -1.5 -1.0 -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  12*7
b_C = 10 * ones(node,1); 
c_G = 2 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5;
           -1.5 -1.0 -0.5 0 0.5 1 1.5];               % 高斯函数的中心点矢量 维度 IN * MID  6*7
b_G = 10 * ones(node,1);
s_new = [0; 0; 0; 0;0; 0];
s_past = s_new;
inte_s_new = [0; 0; 0; 0;0; 0];
inte_s_past = inte_s_new;
gamam = [5;5;5;5;5;5];         % 对称正定常矩阵    增大系数会增大抖振
gamac = [10;10;10;10;10;10];         % 对称正定常矩阵    增大系数会增大抖振
gamag = [5;5;5;5;5;5];          % 对称正定常矩阵         增大系数会增大抖振
kp = [100 0 0 0 0 0;0 100 0 0 0 0;0 0 100 0 0 0;0 0 0 100 0 0;0 0 0 0 100 0;0 0 0 0 0 100];     % kp>0
ki = [100 0 0 0 0 0;0 100 0 0 0 0;0 0 100 0 0 0;0 0 0 100 0 0;0 0 0 0 100 0;0 0 0 0 0 100];     % ki>0
kr = 2*[0.1 0 0 0 0 0;0 0.1 0 0 0 0;0 0 0.1 0 0 0;0 0 0 0.1 0 0;0 0 0 0 0.1 0;0 0 0 0 0 0.1];    % 鲁棒项 kr>=sup.|d(t)|
xite = [50 0 0 0 0 0;    % 增大系数会增大抖振，但会提高收敛速度
        0 10 0 0 0 0;
        0 0 20 0 0 0;
        0 0 0 10 0 0;
        0 0 0 0 30 0;
        0 0 0 0 0 30];  % xite>0
sizes = simsizes;
sizes.NumContStates  = node*78;   %设置系统连续状态的变量 W V 2*2=4 2*2=4 2*1=2 -- 6*6=36  6*6=36  6*1=6  
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 9;   %设置系统输出的变量
sizes.NumInputs      = 36;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = 0 * ones(node*78,1);            % 系统初始状态变量 代表W和V向量 
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0



function sys = mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s_past inte_s_new gamam gamac gamag kp ki kr xite
% 仿真中应根据网络输入值的有效映射范围来设计 c和b 从而保证有效的高斯映射  不合适的b或c均会导致结果不正确
% 角度跟踪指令
qd1 = u(1);
qd2 = u(2);   
qd3 = u(3);
qd4 = u(4); 
qd5 = u(5);
qd6 = u(6); 

% 角速度跟踪指令
dqd1 = u(7);
dqd2 = u(8);   
dqd3 = u(9);
dqd4 = u(10); 
dqd5 = u(11);
dqd6 = u(12); 

% 角加速度跟踪指令
ddqd1 = u(13);
ddqd2 = u(14);   
ddqd3 = u(15);
ddqd4 = u(16); 
ddqd5 = u(17);
ddqd6 = u(18); 

q1 = u(19);
q2 = u(20);
q3 = u(21);
q4 = u(22);
q5 = u(23);
q6 = u(24);

dq1 = u(25);
dq2 = u(26);
dq3 = u(27);
dq4 = u(28);
dq5 = u(29);
dq6 = u(30);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
e3 = qd3 - q3;      % e = qd - q
e4 = qd4 - q4;
e5 = qd5 - q5;      % e = qd - q
e6 = qd6 - q6;

de1 = dqd1 - dq1;
de2 = dqd2 - dq2;
de3 = dqd3 - dq3;
de4 = dqd4 - dq4;
de5 = dqd5 - dq5;
de6 = dqd6 - dq6;

e = [e1; e2; e3; e4; e5; e6];
de = [de1; de2; de3; de4; de5; de6];

dq = [dq1; dq2; dq3; dq4; dq5; dq6];
q = [q1; q2; q3; q4; q5; q6];
dqd = [dqd1; dqd2; dqd3; dqd4; dqd5; dqd6;];
ddqd = [ddqd1; ddqd2; ddqd3; ddqd4; ddqd5; ddqd6];

% 参数的定义
belta = 1;
p = 5;
q = 3;
s_new = xite * e + de;
% s_new = e + 1/belta*abs(de)^(p/q);
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% 神经网络的输入
input1 = [q1; q2; q3; q4; q5; q6];  %6*1
% ------------------------------------M矩阵的径向基函数
h_M11 = zeros(node , 1); h_M12 = zeros(node , 1); h_M13 = zeros(node , 1); h_M14 = zeros(node , 1); h_M15 = zeros(node , 1); h_M16 = zeros(node , 1);
h_M21 = zeros(node , 1); h_M22 = zeros(node , 1); h_M23 = zeros(node , 1); h_M24 = zeros(node , 1); h_M25 = zeros(node , 1); h_M26 = zeros(node , 1);
h_M31 = zeros(node , 1); h_M32 = zeros(node , 1); h_M33 = zeros(node , 1); h_M34 = zeros(node , 1); h_M35 = zeros(node , 1); h_M36 = zeros(node , 1);
h_M41 = zeros(node , 1); h_M42 = zeros(node , 1); h_M43 = zeros(node , 1); h_M44 = zeros(node , 1); h_M45 = zeros(node , 1); h_M46 = zeros(node , 1);
h_M51 = zeros(node , 1); h_M52 = zeros(node , 1); h_M53 = zeros(node , 1); h_M54 = zeros(node , 1); h_M55 = zeros(node , 1); h_M56 = zeros(node , 1);
h_M61 = zeros(node , 1); h_M62 = zeros(node , 1); h_M63 = zeros(node , 1); h_M64 = zeros(node , 1); h_M65 = zeros(node , 1); h_M66 = zeros(node , 1);

for i =1:node
    h_M11(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M12(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M13(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M14(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M15(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M16(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M21(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M22(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
    h_M23(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M24(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M25(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M26(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M31(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M32(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M33(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M34(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
    h_M35(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M36(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M41(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M42(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M43(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M44(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M45(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M46(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M51(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M52(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
    h_M53(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M54(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M55(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M56(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M61(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M62(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M63(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M64(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M65(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
    h_M66(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
end

W_M11 = x(1:node);             W_M12 = x(node*1+1: node*2);  W_M13 = x(node*2+1: node*3);  W_M14 = x(node*3+1: node*4);  W_M15 = x(node*4+1: node*5);  W_M16 = x(node*5+1: node*6);  
W_M21 = x(node*6+1: node*7);   W_M22 = x(node*7+1: node*8);  W_M23 = x(node*8+1: node*9);  W_M24 = x(node*9+1: node*10); W_M25 = x(node*10+1: node*11);W_M26 = x(node*11+1: node*12);
W_M31 = x(node*12+1: node*13); W_M32 = x(node*13+1: node*14);W_M33 = x(node*14+1: node*15);W_M34 = x(node*15+1: node*16);W_M35 = x(node*16+1: node*17);W_M36 = x(node*17+1: node*18);
W_M41 = x(node*18+1: node*19); W_M42 = x(node*19+1: node*20);W_M43 = x(node*20+1: node*21);W_M44 = x(node*21+1: node*22);W_M45 = x(node*22+1: node*23);W_M46 = x(node*23+1: node*24);
W_M51 = x(node*24+1: node*25); W_M52 = x(node*25+1: node*26);W_M53 = x(node*26+1: node*27);W_M54 = x(node*27+1: node*28);W_M55 = x(node*28+1: node*29);W_M56 = x(node*29+1: node*30);
W_M61 = x(node*30+1: node*31); W_M62 = x(node*31+1: node*32);W_M63 = x(node*32+1: node*33);W_M64 = x(node*33+1: node*34);W_M65 = x(node*34+1: node*35);W_M66 = x(node*35+1: node*36);

% W_M权值的自适应律
dw_M11 = gamam(1) * h_M11 * ddqr(1) * s_new(1); 
dw_M12 = gamam(1) * h_M12 * ddqr(2) * s_new(1);
dw_M13 = gamam(1) * h_M13 * ddqr(3) * s_new(1);
dw_M14 = gamam(1) * h_M14 * ddqr(4) * s_new(1);
dw_M15 = gamam(1) * h_M15 * ddqr(5) * s_new(1);
dw_M16 = gamam(1) * h_M16 * ddqr(6) * s_new(1);

dw_M21 = gamam(2) * h_M21 * ddqr(1) * s_new(2);
dw_M22 = gamam(2) * h_M22 * ddqr(2) * s_new(2);
dw_M23 = gamam(2) * h_M23 * ddqr(3) * s_new(2);
dw_M24 = gamam(2) * h_M24 * ddqr(4) * s_new(2);
dw_M25 = gamam(2) * h_M25 * ddqr(5) * s_new(2);
dw_M26 = gamam(2) * h_M26 * ddqr(6) * s_new(2);

dw_M31 = gamam(3) * h_M31 * ddqr(1) * s_new(3);
dw_M32 = gamam(3) * h_M32 * ddqr(2) * s_new(3);
dw_M33 = gamam(3) * h_M33 * ddqr(3) * s_new(3);
dw_M34 = gamam(3) * h_M34 * ddqr(4) * s_new(3);
dw_M35 = gamam(3) * h_M35 * ddqr(5) * s_new(3);
dw_M36 = gamam(3) * h_M36 * ddqr(6) * s_new(3);

dw_M41 = gamam(4) * h_M41 * ddqr(1) * s_new(4);
dw_M42 = gamam(4) * h_M42 * ddqr(2) * s_new(4);
dw_M43 = gamam(4) * h_M43 * ddqr(3) * s_new(4);
dw_M44 = gamam(4) * h_M44 * ddqr(4) * s_new(4);
dw_M45 = gamam(4) * h_M45 * ddqr(5) * s_new(4);
dw_M46 = gamam(4) * h_M46 * ddqr(6) * s_new(4);

dw_M51 = gamam(5) * h_M51 * ddqr(1) * s_new(5);
dw_M52 = gamam(5) * h_M52 * ddqr(2) * s_new(5);
dw_M53 = gamam(5) * h_M53 * ddqr(3) * s_new(5);
dw_M54 = gamam(5) * h_M54 * ddqr(4) * s_new(5);
dw_M55 = gamam(5) * h_M55 * ddqr(5) * s_new(5);
dw_M56 = gamam(5) * h_M56 * ddqr(6) * s_new(5);

dw_M61 = gamam(6) * h_M61 * ddqr(1) * s_new(6);
dw_M62 = gamam(6) * h_M62 * ddqr(2) * s_new(6);
dw_M63 = gamam(6) * h_M63 * ddqr(3) * s_new(6);
dw_M64 = gamam(6) * h_M64 * ddqr(4) * s_new(6);
dw_M65 = gamam(6) * h_M65 * ddqr(5) * s_new(6);
dw_M66 = gamam(6) * h_M66 * ddqr(6) * s_new(6);

for i = 1:node
    sys(i) = dw_M11(i);         sys(i+node*1) = dw_M12(i); sys(i+node*2) = dw_M13(i); sys(i+node*3) = dw_M14(i);   sys(i+node*4) = dw_M15(i); sys(i+node*5) = dw_M16(i);
    sys(i+node*6) = dw_M21(i);  sys(i+node*7) = dw_M22(i); sys(i+node*8) = dw_M23(i); sys(i+node*9) = dw_M24(i);   sys(i+node*10) = dw_M25(i);sys(i+node*11) = dw_M26(i);
    sys(i+node*12) = dw_M31(i); sys(i+node*13) = dw_M32(i);sys(i+node*14) = dw_M33(i);sys(i+node*15) = dw_M34(i);  sys(i+node*16) = dw_M35(i);sys(i+node*17) = dw_M36(i);
    sys(i+node*18) = dw_M41(i); sys(i+node*19) = dw_M42(i);sys(i+node*20) = dw_M43(i);sys(i+node*21) = dw_M44(i);  sys(i+node*22) = dw_M45(i);sys(i+node*23) = dw_M46(i);
    sys(i+node*24) = dw_M51(i); sys(i+node*25) = dw_M52(i);sys(i+node*26) = dw_M53(i);sys(i+node*27) = dw_M54(i);  sys(i+node*28) = dw_M55(i);sys(i+node*29) = dw_M56(i);
    sys(i+node*30) = dw_M61(i); sys(i+node*31) = dw_M62(i);sys(i+node*32) = dw_M63(i);sys(i+node*33) = dw_M64(i);  sys(i+node*34) = dw_M65(i);sys(i+node*35) = dw_M66(i);
end

% ----------------------------------------C矩阵的径向基函数
input2 = [q1;q2;q3;q4;q5;q6;dq1;dq2;dq3;dq4;dq5;dq6]; %12*1
h_C11 = zeros(node , 1); h_C12 = zeros(node , 1); h_C13 = zeros(node , 1); h_C14 = zeros(node , 1); h_C15 = zeros(node , 1); h_C16 = zeros(node , 1);
h_C21 = zeros(node , 1); h_C22 = zeros(node , 1); h_C23 = zeros(node , 1); h_C24 = zeros(node , 1); h_C25 = zeros(node , 1); h_C26 = zeros(node , 1);
h_C31 = zeros(node , 1); h_C32 = zeros(node , 1); h_C33 = zeros(node , 1); h_C34 = zeros(node , 1); h_C35 = zeros(node , 1); h_C36 = zeros(node , 1);
h_C41 = zeros(node , 1); h_C42 = zeros(node , 1); h_C43 = zeros(node , 1); h_C44 = zeros(node , 1); h_C45 = zeros(node , 1); h_C46 = zeros(node , 1);
h_C51 = zeros(node , 1); h_C52 = zeros(node , 1); h_C53 = zeros(node , 1); h_C54 = zeros(node , 1); h_C55 = zeros(node , 1); h_C56 = zeros(node , 1);
h_C61 = zeros(node , 1); h_C62 = zeros(node , 1); h_C63 = zeros(node , 1); h_C64 = zeros(node , 1); h_C65 = zeros(node , 1); h_C66 = zeros(node , 1);

for i =1:node
    h_C11(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C12(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C13(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C14(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C15(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C16(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C21(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C22(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C23(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C24(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C25(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C26(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C31(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C32(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C33(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C34(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C35(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C36(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C41(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C42(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C43(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C44(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C45(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C46(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C51(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C52(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C53(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C54(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C55(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C56(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C61(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C62(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C63(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C64(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C65(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C66(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
end

W_C11 = x(1+node*36:node*37);  W_C12 = x(node*37+1: node*38);  W_C13 = x(node*38+1: node*39);  W_C14 = x(node*39+1: node*40);  W_C15 = x(node*40+1: node*41);  W_C16 = x(node*41+1: node*42);  
W_C21 = x(node*42+1: node*43); W_C22 = x(node*43+1: node*44);  W_C23 = x(node*44+1: node*45);  W_C24 = x(node*45+1: node*46); W_C25 = x(node*46+1: node*47);W_C26 = x(node*47+1: node*48);
W_C31 = x(node*48+1: node*49); W_C32 = x(node*49+1: node*50);W_C33 = x(node*50+1: node*51);W_C34 = x(node*51+1: node*52);W_C35 = x(node*52+1: node*53);W_C36 = x(node*53+1: node*54);
W_C41 = x(node*54+1: node*55); W_C42 = x(node*55+1: node*56);W_C43 = x(node*56+1: node*57);W_C44 = x(node*57+1: node*58);W_C45 = x(node*58+1: node*59);W_C46 = x(node*59+1: node*60);
W_C51 = x(node*60+1: node*61); W_C52 = x(node*61+1: node*62);W_C53 = x(node*62+1: node*63);W_C54 = x(node*63+1: node*64);W_C55 = x(node*64+1: node*65);W_C56 = x(node*65+1: node*66);
W_C61 = x(node*66+1: node*67); W_C62 = x(node*67+1: node*68);W_C63 = x(node*68+1: node*69);W_C64 = x(node*69+1: node*70);W_C65 = x(node*70+1: node*71);W_C66 = x(node*71+1: node*72);

% W_C权值的自适应律
dw_C11 = gamac(1) * h_C11 * dqr(1) * s_new(1);
dw_C12 = gamac(1) * h_C12 * ddqr(2) * s_new(1);
dw_C13 = gamac(1) * h_C13 * ddqr(3) * s_new(1);
dw_C14 = gamac(1) * h_C14 * ddqr(4) * s_new(1);
dw_C15 = gamac(1) * h_C15 * ddqr(5) * s_new(1);
dw_C16 = gamac(1) * h_C16 * ddqr(6) * s_new(1);

dw_C21 = gamac(2) * h_C21 * dqr(1) * s_new(2);
dw_C22 = gamac(2) * h_C22 * ddqr(2) * s_new(2);
dw_C23 = gamac(2) * h_C23 * ddqr(3) * s_new(2);
dw_C24 = gamac(2) * h_C24 * ddqr(4) * s_new(2);
dw_C25 = gamac(2) * h_C25 * ddqr(5) * s_new(2);
dw_C26 = gamac(2) * h_C26 * ddqr(6) * s_new(2);

dw_C31 = gamac(3) * h_C31 * dqr(1) * s_new(3);
dw_C32 = gamac(3) * h_C32 * ddqr(2) * s_new(3);
dw_C33 = gamac(3) * h_C33 * ddqr(3) * s_new(3);
dw_C34 = gamac(3) * h_C34 * ddqr(4) * s_new(3);
dw_C35 = gamac(3) * h_C35 * ddqr(5) * s_new(3);
dw_C36 = gamac(3) * h_C36 * ddqr(6) * s_new(3);

dw_C41 = gamac(4) * h_C41 * dqr(1) * s_new(4);
dw_C42 = gamac(4) * h_C42 * ddqr(2) * s_new(4);
dw_C43 = gamac(4) * h_C43 * ddqr(3) * s_new(4);
dw_C44 = gamac(4) * h_C44 * ddqr(4) * s_new(4);
dw_C45 = gamac(4) * h_C45 * ddqr(5) * s_new(4);
dw_C46 = gamac(4) * h_C46 * ddqr(6) * s_new(4);

dw_C51 = gamac(5) * h_C51 * dqr(1) * s_new(5);
dw_C52 = gamac(5) * h_C52 * ddqr(2) * s_new(5);
dw_C53 = gamac(5) * h_C53 * ddqr(3) * s_new(5);
dw_C54 = gamac(5) * h_C54 * ddqr(4) * s_new(5);
dw_C55 = gamac(5) * h_C55 * ddqr(5) * s_new(5);
dw_C56 = gamac(5) * h_C56 * ddqr(6) * s_new(5);

dw_C61 = gamac(6) * h_C61 * dqr(1) * s_new(6);
dw_C62 = gamac(6) * h_C62 * ddqr(2) * s_new(6);
dw_C63 = gamac(6) * h_C63 * ddqr(3) * s_new(6);
dw_C64 = gamac(6) * h_C64 * ddqr(4) * s_new(6);
dw_C65 = gamac(6) * h_C65 * ddqr(5) * s_new(6);
dw_C66 = gamac(6) * h_C66 * ddqr(6) * s_new(6);

for i = 1:node
    sys(i+node*36) = dw_C11(i);sys(i+node*37) = dw_C12(i);sys(i+node*38) = dw_C13(i);sys(i+node*39) = dw_C14(i);sys(i+node*40) = dw_C15(i);sys(i+node*41) = dw_C16(i);
    sys(i+node*42) = dw_C21(i);sys(i+node*43) = dw_C22(i);sys(i+node*44) = dw_C23(i);sys(i+node*45) = dw_C24(i);sys(i+node*46) = dw_C25(i);sys(i+node*47) = dw_C26(i);
    sys(i+node*48) = dw_C31(i);sys(i+node*49) = dw_C32(i);sys(i+node*50) = dw_C33(i);sys(i+node*51) = dw_C34(i);sys(i+node*52) = dw_C35(i);sys(i+node*53) = dw_C36(i);
    sys(i+node*54) = dw_C41(i);sys(i+node*55) = dw_C42(i);sys(i+node*56) = dw_C43(i);sys(i+node*57) = dw_C44(i);sys(i+node*58) = dw_C45(i);sys(i+node*59) = dw_C46(i);
    sys(i+node*60) = dw_C51(i);sys(i+node*61) = dw_C52(i);sys(i+node*62) = dw_C53(i);sys(i+node*63) = dw_C54(i);sys(i+node*64) = dw_C55(i);sys(i+node*65) = dw_C56(i);
    sys(i+node*66) = dw_C61(i);sys(i+node*67) = dw_C62(i);sys(i+node*68) = dw_C63(i);sys(i+node*69) = dw_C64(i);sys(i+node*70) = dw_C65(i);sys(i+node*71) = dw_C66(i);
end

% ----------------------------------------G矩阵的径向基函数
input3 = [q1; q2; q3; q4; q5; q6];  %6*1
h_G11 = zeros(node , 1); h_G12 = zeros(node , 1); h_G13 = zeros(node , 1); h_G14 = zeros(node , 1); h_G15 = zeros(node , 1); h_G16 = zeros(node , 1);

for i =1:node
    h_G11(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G12(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    h_G13(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G14(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    h_G15(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G16(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    
end
W_G11 = x(node*72+1:node*73);     % 7*1
W_G12 = x(node*73+1: node*74);
W_G13 = x(node*74+1:node*75);     % 7*1
W_G14 = x(node*75+1: node*76);
W_G15 = x(node*76+1:node*77);     % 7*1
W_G16 = x(node*77+1: node*78);

% W_C权值的自适应律
dw_G11 = gamag(1) * h_G11 * s_new(1);
dw_G12 = gamag(2) * h_G12 * s_new(2);
dw_G13 = gamag(3) * h_G13 * s_new(3);
dw_G14 = gamag(4) * h_G14 * s_new(4);
dw_G15 = gamag(5) * h_G15 * s_new(5);
dw_G16 = gamag(6) * h_G16 * s_new(6);

for i = 1:node
    sys(i+node*72) = dw_G11(i);
    sys(i+node*73) = dw_G12(i);
    sys(i+node*74) = dw_G13(i);
    sys(i+node*75) = dw_G14(i);
    sys(i+node*76) = dw_G15(i);
    sys(i+node*77) = dw_G16(i);
end

function sys = mdlOutputs(t,x,u)   %产生（传递）系统输出
global c_M b_M c_C b_C c_G b_G node s_new s_past inte_s_past inte_s_new gamam gamac gamag kp ki kr xite
% 角度跟踪指令
qd1 = u(1);
qd2 = u(2);   
qd3 = u(3);
qd4 = u(4); 
qd5 = u(5);
qd6 = u(6); 

% 角速度跟踪指令
dqd1 = u(7);
dqd2 = u(8);   
dqd3 = u(9);
dqd4 = u(10); 
dqd5 = u(11);
dqd6 = u(12); 

% 角加速度跟踪指令
ddqd1 = u(13);
ddqd2 = u(14);   
ddqd3 = u(15);
ddqd4 = u(16); 
ddqd5 = u(17);
ddqd6 = u(18); 

q1 = u(19);
q2 = u(20);
q3 = u(21);
q4 = u(22);
q5 = u(23);
q6 = u(24);

dq1 = u(25);
dq2 = u(26);
dq3 = u(27);
dq4 = u(28);
dq5 = u(29);
dq6 = u(30);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
e3 = qd3 - q3;      % e = qd - q
e4 = qd4 - q4;
e5 = qd5 - q5;      % e = qd - q
e6 = qd6 - q6;

de1 = dqd1 - dq1;
de2 = dqd2 - dq2;
de3 = dqd3 - dq3;
de4 = dqd4 - dq4;
de5 = dqd5 - dq5;
de6 = dqd6 - dq6;

e = [e1; e2; e3; e4; e5; e6];
de = [de1; de2; de3; de4; de5; de6];

dq = [dq1; dq2; dq3; dq4; dq5; dq6];
q = [q1; q2; q3; q4; q5; q6];
dqd = [dqd1; dqd2; dqd3; dqd4; dqd5; dqd6;];
ddqd = [ddqd1; ddqd2; ddqd3; ddqd4; ddqd5; ddqd6];

% 滑模面
s_new = xite * e + de;
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% ------------------------------------M矩阵的径向基函数
% 神经网络的输入
input1 = [q1; q2; q3; q4; q5; q6];  %6*1
% ------------------------------------M矩阵的径向基函数
h_M11 = zeros(node , 1); h_M12 = zeros(node , 1); h_M13 = zeros(node , 1); h_M14 = zeros(node , 1); h_M15 = zeros(node , 1); h_M16 = zeros(node , 1);
h_M21 = zeros(node , 1); h_M22 = zeros(node , 1); h_M23 = zeros(node , 1); h_M24 = zeros(node , 1); h_M25 = zeros(node , 1); h_M26 = zeros(node , 1);
h_M31 = zeros(node , 1); h_M32 = zeros(node , 1); h_M33 = zeros(node , 1); h_M34 = zeros(node , 1); h_M35 = zeros(node , 1); h_M36 = zeros(node , 1);
h_M41 = zeros(node , 1); h_M42 = zeros(node , 1); h_M43 = zeros(node , 1); h_M44 = zeros(node , 1); h_M45 = zeros(node , 1); h_M46 = zeros(node , 1);
h_M51 = zeros(node , 1); h_M52 = zeros(node , 1); h_M53 = zeros(node , 1); h_M54 = zeros(node , 1); h_M55 = zeros(node , 1); h_M56 = zeros(node , 1);
h_M61 = zeros(node , 1); h_M62 = zeros(node , 1); h_M63 = zeros(node , 1); h_M64 = zeros(node , 1); h_M65 = zeros(node , 1); h_M66 = zeros(node , 1);

for i =1:node
    h_M11(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M12(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M13(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M14(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M15(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M16(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M21(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M22(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2));
    h_M23(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M24(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M25(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M26(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M31(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M32(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M33(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M34(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M35(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M36(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M41(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M42(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M43(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M44(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M45(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M46(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M51(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M52(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M53(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M54(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M55(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M56(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    
    h_M61(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M62(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M63(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M64(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M65(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
    h_M66(i) = exp(-(norm(input1 - c_M(:,i))^2) / (b_M(i)^2)); 
end

W_M11 = x(1:node);             W_M12 = x(node*1+1: node*2);  W_M13 = x(node*2+1: node*3);  W_M14 = x(node*3+1: node*4);  W_M15 = x(node*4+1: node*5);  W_M16 = x(node*5+1: node*6);  
W_M21 = x(node*6+1: node*7);   W_M22 = x(node*7+1: node*8);  W_M23 = x(node*8+1: node*9);  W_M24 = x(node*9+1: node*10); W_M25 = x(node*10+1: node*11);W_M26 = x(node*11+1: node*12);
W_M31 = x(node*12+1: node*13); W_M32 = x(node*13+1: node*14);W_M33 = x(node*14+1: node*15);W_M34 = x(node*15+1: node*16);W_M35 = x(node*16+1: node*17);W_M36 = x(node*17+1: node*18);
W_M41 = x(node*18+1: node*19); W_M42 = x(node*19+1: node*20);W_M43 = x(node*20+1: node*21);W_M44 = x(node*21+1: node*22);W_M45 = x(node*22+1: node*23);W_M46 = x(node*23+1: node*24);
W_M51 = x(node*24+1: node*25); W_M52 = x(node*25+1: node*26);W_M53 = x(node*26+1: node*27);W_M54 = x(node*27+1: node*28);W_M55 = x(node*28+1: node*29);W_M56 = x(node*29+1: node*30);
W_M61 = x(node*30+1: node*31); W_M62 = x(node*31+1: node*32);W_M63 = x(node*32+1: node*33);W_M64 = x(node*33+1: node*34);W_M65 = x(node*34+1: node*35);W_M66 = x(node*35+1: node*36);

% ----------------------------------------C矩阵的径向基函数
input2 = [q1;q2;q3;q4;q5;q6;dq1;dq2;dq3;dq4;dq5;dq6]; %12*1
h_C11 = zeros(node , 1); h_C12 = zeros(node , 1); h_C13 = zeros(node , 1); h_C14 = zeros(node , 1); h_C15 = zeros(node , 1); h_C16 = zeros(node , 1);
h_C21 = zeros(node , 1); h_C22 = zeros(node , 1); h_C23 = zeros(node , 1); h_C24 = zeros(node , 1); h_C25 = zeros(node , 1); h_C26 = zeros(node , 1);
h_C31 = zeros(node , 1); h_C32 = zeros(node , 1); h_C33 = zeros(node , 1); h_C34 = zeros(node , 1); h_C35 = zeros(node , 1); h_C36 = zeros(node , 1);
h_C41 = zeros(node , 1); h_C42 = zeros(node , 1); h_C43 = zeros(node , 1); h_C44 = zeros(node , 1); h_C45 = zeros(node , 1); h_C46 = zeros(node , 1);
h_C51 = zeros(node , 1); h_C52 = zeros(node , 1); h_C53 = zeros(node , 1); h_C54 = zeros(node , 1); h_C55 = zeros(node , 1); h_C56 = zeros(node , 1);
h_C61 = zeros(node , 1); h_C62 = zeros(node , 1); h_C63 = zeros(node , 1); h_C64 = zeros(node , 1); h_C65 = zeros(node , 1); h_C66 = zeros(node , 1);

for i =1:node
    h_C11(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C12(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C13(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C14(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C15(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C16(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C21(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C22(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C23(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C24(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C25(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C26(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C31(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C32(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C33(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C34(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C35(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C36(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C41(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C42(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C43(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C44(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C45(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C46(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C51(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C52(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C53(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C54(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C55(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C56(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    
    h_C61(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2));
    h_C62(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C63(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C64(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C65(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
    h_C66(i) = exp(-(norm(input2 - c_C(:,i))^2) / (b_C(i)^2)); 
end

W_C11 = x(node*36+1: node*37); W_C12 = x(node*37+1: node*38);W_C13 = x(node*38+1: node*39);W_C14 = x(node*39+1: node*40);W_C15 = x(node*40+1: node*41);W_C16 = x(node*41+1: node*42);  
W_C21 = x(node*42+1: node*43); W_C22 = x(node*43+1: node*44);W_C23 = x(node*44+1: node*45);W_C24 = x(node*45+1: node*46);W_C25 = x(node*46+1: node*47);W_C26 = x(node*47+1: node*48);
W_C31 = x(node*48+1: node*49); W_C32 = x(node*49+1: node*50);W_C33 = x(node*50+1: node*51);W_C34 = x(node*51+1: node*52);W_C35 = x(node*52+1: node*53);W_C36 = x(node*53+1: node*54);
W_C41 = x(node*54+1: node*55); W_C42 = x(node*55+1: node*56);W_C43 = x(node*56+1: node*57);W_C44 = x(node*57+1: node*58);W_C45 = x(node*58+1: node*59);W_C46 = x(node*59+1: node*60);
W_C51 = x(node*60+1: node*61); W_C52 = x(node*61+1: node*62);W_C53 = x(node*62+1: node*63);W_C54 = x(node*63+1: node*64);W_C55 = x(node*64+1: node*65);W_C56 = x(node*65+1: node*66);
W_C61 = x(node*66+1: node*67); W_C62 = x(node*67+1: node*68);W_C63 = x(node*68+1: node*69);W_C64 = x(node*69+1: node*70);W_C65 = x(node*70+1: node*71);W_C66 = x(node*71+1: node*72);

% ----------------------------------------G矩阵的径向基函数
input3 = [q1; q2; q3; q4; q5; q6];  %6*1
h_G11 = zeros(node , 1); h_G12 = zeros(node , 1); h_G13 = zeros(node , 1); h_G14 = zeros(node , 1); h_G15 = zeros(node , 1); h_G16 = zeros(node , 1);

for i =1:node
    h_G11(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G12(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    h_G13(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G14(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    h_G15(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2));
    h_G16(i) = exp(-(norm(input3 - c_G(:,i))^2) / (b_G(i)^2)); 
    
end
W_G11 = x(node*72+1:node*73);     % 7*1
W_G12 = x(node*73+1: node*74);
W_G13 = x(node*74+1:node*75);     % 7*1
W_G14 = x(node*75+1: node*76);
W_G15 = x(node*76+1:node*77);     % 7*1
W_G16 = x(node*77+1: node*78);

% 神经网络的输出
fx1 = [W_M11'*h_M11  W_M12'*h_M12 W_M13'*h_M13  W_M14'*h_M14 W_M15'*h_M15  W_M16'*h_M16;
       W_M21'*h_M21  W_M22'*h_M22 W_M23'*h_M23  W_M24'*h_M24 W_M25'*h_M25  W_M26'*h_M26;
       W_M31'*h_M31  W_M32'*h_M32 W_M33'*h_M33  W_M34'*h_M34 W_M35'*h_M35  W_M36'*h_M36;
       W_M41'*h_M41  W_M42'*h_M42 W_M43'*h_M43  W_M44'*h_M44 W_M45'*h_M45  W_M46'*h_M46;
       W_M51'*h_M51  W_M52'*h_M52 W_M53'*h_M53  W_M54'*h_M54 W_M55'*h_M55  W_M56'*h_M56;
       W_M61'*h_M61  W_M62'*h_M62 W_M63'*h_M63  W_M64'*h_M64 W_M65'*h_M65  W_M66'*h_M66];  %  M矩阵 6*6
   
fx2 = [W_C11'*h_C11  W_C12'*h_C12 W_C13'*h_C13  W_C14'*h_C14 W_C15'*h_C15  W_C16'*h_C16;
       W_C21'*h_C21  W_C22'*h_C22 W_C23'*h_C23  W_C24'*h_C24 W_C25'*h_C25  W_C26'*h_C26;
       W_C31'*h_C31  W_C32'*h_C32 W_C33'*h_C33  W_C34'*h_C34 W_C35'*h_C35  W_C36'*h_C36;
       W_C41'*h_C41  W_C42'*h_C42 W_C43'*h_C43  W_C44'*h_C44 W_C45'*h_C45  W_C46'*h_C46;
       W_C51'*h_C51  W_C52'*h_C52 W_C53'*h_C53  W_C54'*h_C54 W_C55'*h_C55  W_C56'*h_C56;
       W_C61'*h_C61  W_C62'*h_C62 W_C63'*h_C63  W_C64'*h_C64 W_C65'*h_C65  W_C66'*h_C66];  % N矩阵 6*6
   
fx3 = [W_G11'*h_G11; W_G12'*h_G12; W_G13'*h_G13; W_G14'*h_G14; W_G15'*h_G15; W_G16'*h_G16;];

M_refer = fx1;
C_refer = fx2;
G_refer = fx3;

% 名义模型控制律
taum = M_refer*ddqr + C_refer*dqr + G_refer;    % 6*6
% 鲁棒项
% taur = kr*sign(s_new);   % 因为控制率中符号函数的存在 导致系统状态存在抖振，在滑模面上也有抖振现象
% 用饱和函数中的双曲正切函数代替,但会降低系统的跟踪性能，可以增大ce+de中的c
taur = kr * (exp(s_new)-exp(-s_new))/(exp(s_new)+exp(-s_new));  

dt = 0.05;
intes1 = u(31);
intes2 = u(32);
intes3 = u(33);
intes4 = u(34);
intes5 = u(35);
intes6 = u(36);
intes = [intes1;intes2;intes3;intes4;intes5;intes6];

inte_s_new = inte_s_past + (s_past + s_new)*dt/2;
tau = taum + kp*s_new + ki*inte_s_new + taur ;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = tau(3);
sys(4) = tau(4);
sys(5) = tau(5);
sys(6) = tau(6);

sys(7) = norm(fx1);
sys(8) = norm(fx2);
sys(9) = norm(fx3);

s_past = s_new;
inte_s_past = inte_s_new;









