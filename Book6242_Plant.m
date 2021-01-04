function [sys,x0,str,ts] = Book6242_Plant(t,x,u,flag)
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
x0  = [0.09 -0.09 0 0];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
tau1 = u(1);    %����1
tau2 = u(2);    %����2

q1 = x(1);      % �ؽڽ�һ
q2 = x(2);      % �ؽڽǶ�
dq1 = x(3);     % �ؽڽ��ٶ�һ
dq2 = x(4);     % �ؽڽ��ٶ�һ

% �����Ķ���
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;
p5 = 0.87;
g = 9.8;

M = [p1+p2+2*p3*cos(q2) p2+p3*cos(q2)
     p2+p3*cos(q2) p2];
C = [-p3*dq2*sin(q2)   -p3*(dq1 + dq2)*sin(q2);
     p3*dq1*sin(q2) 0];
G = [p4*g*cos(q1)+p5*g*cos(q1+q2);
     p5*g*cos(q1+q2)];
 
% �ⲿ����
F = [2*sign(dq1); 2*sign(dq2)];
taud = [2*sin(t); 2*sin(t)];
tau = [tau1; tau2];

ddq = inv(M) * (tau - C*[dq1; dq2] - G - F - taud);

sys(1) = x(3);   
sys(2) = x(4);
sys(3) = ddq(1);
sys(4) = ddq(2);


function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
sys(1) = x(1);   %q1
sys(2) = x(2);   %q2
sys(3) = x(3);   %dq1
sys(4) = x(4);   %dq2



