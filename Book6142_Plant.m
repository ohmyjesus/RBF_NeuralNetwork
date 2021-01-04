function [sys,x0,str,ts] = Book6142_Plant(t,x,u,flag)
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
global ddq1 ddq2
ddq1 = 0;
ddq2 = 0;

function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global ddq1 ddq2
tau1 = u(1);    %����1
tau2 = u(2);    %����2

q1 = x(1);      % �ؽڽ�һ
q2 = x(2);      % �ؽڽǶ�
dq1 = x(3);     % �ؽڽ��ٶ�һ
dq2 = x(4);     % �ؽڽ��ٶ�һ

% �����Ķ���
v = 13.33;
a1 = 8.98;
a2 = 8.75;
g = 9.8;
d1 = 2;
d2 = 3;
d3 = 6;

% �Ƕȸ���ָ��
qd1 = 1+0.2*sin(0.5*pi*t);
qd2 = 1-0.2*cos(0.5*pi*t);
dqd1 = 0.1*pi*cos(0.5*pi*t);
dqd2 = 0.1*pi*sin(0.5*pi*t);

M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
     a1+a2*cos(q2)  a1];
C = [-a2*dq2*sin(q2)   -a2*(dq1 + dq2)*sin(q2);
     a2*dq1*sin(q2) 0];
G = [15*g*cos(q1)+8.75*g*cos(q1+q2);
     8.75*g*cos(q1+q2)];
deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;
% �ⲿ����
e1 = q1 - qd1;
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
ddq = [ddq1; ddq2];

d = d1 + d2 * norm([e1;e2]) + d3 * norm([de1; de2]);
f = deltam * ddq + deltac * [dq1; dq2] + deltag + d;
tau = [tau1; tau2];

ddq = inv(M) * (tau - C*[dq1; dq2] - G + f);

sys(1) = x(3);   
sys(2) = x(4);
sys(3) = ddq(1);
sys(4) = ddq(2);


function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
sys(1) = x(1);   %q1
sys(2) = x(2);   %q2
sys(3) = x(3);   %dq1
sys(4) = x(4);   %dq2



