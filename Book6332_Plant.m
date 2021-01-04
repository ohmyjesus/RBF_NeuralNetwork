function [sys,x0,str,ts] = Book6332_Plant(t,x,u,flag)
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
sizes.NumOutputs     = 6;   %����ϵͳ����ı���
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 0;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [0 0 0 0];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
% �Ƕȸ���ָ��
% qd1 = sin(t);
% qd2 = sin(t);
% dqd1 = cos(t);
% dqd2 = cos(t);
tau1 = u(1);    %����1
tau2 = u(2);    %����2

q1 = x(1);      % �ؽڽ�һ
q2 = x(2);      % �ؽڽǶ�
dq1 = x(3);     % �ؽڽ��ٶ�һ
dq2 = x(4);     % �ؽڽ��ٶ�һ
q = [q1; q2];
dq = [dq1; dq2];

% �����Ķ���
m1 = 1;
m2 = 1.5;
r1 = 1;
r2 = 0.8;
M11 = (m1 + m2)*r1^2 + m2*r2^2 + 2*m2*r1*r2*cos(q2);
M12 = m2*r2^2 + m2*r1*r2*cos(q2);
M21 = M12;
M22 = m2 * r2^2;
V12 = m2*r1*sin(q2);
G1 = (m1+m2)*r1*cos(q2) + m2*r2*cos(q1+q2);
G2 = m2*r2*cos(q1+q2);

M = [M11 M12;
     M21 M22];
V = [-V12*dq2  -V12*(dq1+dq2);
     V12*q1     0];
G = [G1; G2];
D = [10*dq1 + 30*sign(dq1); 10*dq2 + 30*sign(dq2)];

tau = [tau1; tau2];

ddq = inv(M) * (tau - D - G - V*dq);

sys(1) = x(3);   
sys(2) = x(4);
sys(3) = ddq(1);
sys(4) = ddq(2);


function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
tau1 = u(1);    %����1
tau2 = u(2);    %����2

q1 = x(1);      % �ؽڽ�һ
q2 = x(2);      % �ؽڽǶ�
dq1 = x(3);     % �ؽڽ��ٶ�һ
dq2 = x(4);     % �ؽڽ��ٶ�һ
q = [q1; q2];
dq = [dq1; dq2];

% �����Ķ���
m1 = 1;
m2 = 1.5;
r1 = 1;
r2 = 0.8;
M11 = (m1 + m2)*r1^2 + m2*r2^2 + 2*m2*r1*r2*cos(q2);
M12 = m2*r2^2 + m2*r1*r2*cos(q2);
M21 = M12;
M22 = m2 * r2^2;
V12 = m2*r1*sin(q2);
G1 = (m1+m2)*r1*cos(q2) + m2*r2*cos(q1+q2);
G2 = m2*r2*cos(q1+q2);

M = [M11 M12;
     M21 M22];
V = [-V12*dq2  -V12*(dq1+dq2);
     V12*q1     0];
G = [G1; G2];
D = [10*dq1 + 30*sign(dq1); 10*dq2 + 30*sign(dq2)];

sys(1) = x(1);   %q1
sys(2) = x(2);   %q2
sys(3) = x(3);   %dq1
sys(4) = x(4);   %dq2
sys(5) = D(1);
sys(6) = D(2);


