function [sys,x0,str,ts] = B2(t,x,u,flag)
switch flag
  case 0 %��ʼ��
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1 %����״̬����
    sys=mdlDerivatives(t,x,u);
  case {2,4,9} %��ɢ״̬���㣬��һ������ʱ�̣���ֹ�����趨
    sys=[];
  case 3 %����źż���
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %ϵͳ�ĳ�ʼ��
sizes = simsizes;
sizes.NumContStates  = 4;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 4;   %����ϵͳ����ı���
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 0;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [0.6 0.3 0.5 0.5];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
persistent ddx1 ddx2
if t == 0
    ddx1 = 0;
    ddx2 = 0;
end
% �Ƕȸ���ָ��
qd1 = 1+0.2*sin(0.5*pi*t);
qd2 = 1-0.2*cos(0.5*pi*t);
dqd1 = 0.1*pi*cos(0.5*pi*t);
dqd2 = 0.1*pi*sin(0.5*pi*t);

e1 = x(1) - qd1;
e2 = x(3) - qd2;
de1 = x(2) - dqd1;
de2 = x(4) - dqd2;

q1 = x(1);
q2 = x(3);
dq1 = x(2);
dq2 = x(4);

% �����Ķ���
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
deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;

d1 = 2;
d2 = 3;
d3 = 6;
d = d1 + d2 * norm([e1;e2]) + d3 * norm([de1; de2]);
tol(1) = u(1);    %����1
tol(2) = u(2);    %����2

dq = [x(2); x(4)];
ddq = [ddx1; ddx2];
f = inv(M) * (deltam * ddq + deltac * dq + deltag + d);

ddx = inv(M) * (tol' - C* dq - G) + f;

sys(1) = x(2);   
sys(2) = ddx(1);
sys(3) = x(4);
sys(4) = ddx(2);
ddx1 = ddx(1);
ddx2 = ddx(2);
 

function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
sys(1) = x(1);   %q1
sys(2) = x(2);   %dq1
sys(3) = x(3);   %q2
sys(4) = x(4);   %dq2



