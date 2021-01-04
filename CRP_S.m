function [sys,x0,str,ts] = CRP_S(t,x,u,flag)
switch flag
      case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
      case {1,2,4,9}
    sys=[];
      case 3
    sys=mdlOutputs(t,x,u);
      otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 30;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;  
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)
global xite
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

s = xite * e + de;

sys(1:6) = s;



