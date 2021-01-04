function [sys,x0,str,ts] = Book6332_Controller(t,x,u,flag)
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
global c b node
% ���������4 - 7 - 2�ṹ  һ�� 4*7*2�ṹ     ����[e1 e2 de1 de2] ��Ӧ��� [tau1 tau2]
node = 7;
c = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  4*7
b = 10 * ones(node,1);  % ��˹�����Ļ���  ά��node * 1  7*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
sizes = simsizes;
sizes.NumContStates  = node*2;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 4;   %����ϵͳ����ı���
sizes.NumInputs      = 8;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0 * ones(node*2,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b node
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
% �Ƕȸ���ָ��
dqd1 =  cos(t);
dqd2 =  cos(t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);

e1 = q1 - qd1;      % e = qd - q
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
e = [e1; e2];
de = [de1 ; de2];

% �����Ķ���
xite = 1500;
alpha = 20;
gama = 0.05;

input = [e; de];
h = zeros(node , 1);   %7*1����
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (b(i)^2)); % 7*1
end

W = [x(1) x(2) x(3)  x(4)  x(5)  x(6)  x(7);
     x(8) x(9) x(10) x(11) x(12) x(13) x(14)]'; % 7*2 

x2_1 = de + alpha * e;

% Ȩֵ������Ӧ��
dw = -xite * x2_1 * h'; % 
% dw = dw';
for i = 1:node
    sys(i) = dw(1,i);
    sys(i+7) = dw(2,i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node
% �Ƕȸ���ָ��
% dqd1 =  cos(t);
% dqd2 =  cos(t);
ddqd1 = -sin(t);
ddqd2 = -sin(t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);
ddq1 = u(7);
ddq2 = u(8);
dqd1 = cos(t);
dqd2 = cos(t);
ddq = [ddq1; ddq2];
ddqd = [ddqd1; ddqd2];
dqd = [dqd1; dqd2];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;
e = [e1; e2];
de = [de1; de2];

% �����Ķ���
xite = 1500;
alpha = 20;
gama = 0.05;
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

input = [e; de];
h = zeros(node , 1);   %7*1����

for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (b(i)^2)); % 7*1
end
W = [x(1) x(2) x(3)  x(4)  x(5)  x(6)  x(7);
     x(8) x(9) x(10) x(11) x(12) x(13) x(14)]'; % 7*2 
 % �������
fx = W' * h;

omiga = M * alpha * de + V * alpha * e;

x2_1 = de + alpha * e;

% ����������
ut = -omiga - 1/(2*gama^2) * x2_1 + fx - 1/2*x2_1;

tau = ut + M * ddqd + V * dqd + G;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx(1);
sys(4) = fx(2);













