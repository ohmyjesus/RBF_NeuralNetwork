function [sys,x0,str,ts] = CRP_i(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 24;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)
% 角速度跟踪指令
dqd1 =  0.1*pi*cos(pi*t);
dqd2 =  0.1*pi*cos(pi*t);
dqd3 =  0.1*pi*cos(pi*t);
dqd4 =  0.1*pi*cos(pi*t);
dqd5 =  0.1*pi*cos(pi*t);
dqd6 =  0.1*pi*cos(pi*t);

% 角加速度跟踪指令
ddqd1 = -0.1*pi*pi*sin(pi*t);
ddqd2 = -0.1*pi*pi*sin(pi*t);
ddqd3 = -0.1*pi*pi*sin(pi*t);
ddqd4 = -0.1*pi*pi*sin(pi*t);
ddqd5 = -0.1*pi*pi*sin(pi*t);
ddqd6 = -0.1*pi*pi*sin(pi*t);
qd1 = u(1);
qd2 = u(2);   
qd3 = u(3);
qd4 = u(4); 
qd5 = u(5);
qd6 = u(6); 

q1 = u(7);
q2 = u(8);
q3 = u(9);
q4 = u(10);
q5 = u(11);
q6 = u(12);

dq1 = u(13);
dq2 = u(14);
dq3 = u(15);
dq4 = u(16);
dq5 = u(17);
dq6 = u(18);

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
xite = 5.0*eye(6);  % xite>0

s = xite * e + de;

sys(1) = s(1);