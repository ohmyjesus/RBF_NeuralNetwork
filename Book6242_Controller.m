function [sys,x0,str,ts] = Book6242_Controller(t,x,u,flag)
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
% ���������4 - 7 - 2�ṹ
node = 7;
c = 1 * [-1.5 -1  -0.5 0 0.5 1 1.5;
         -1.5 -1  -0.5 0 0.5 1 1.5];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*7
b = 10 * ones(node,1);  % ��˹�����Ļ���  ά��node * 1  5*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
sizes = simsizes;
sizes.NumContStates  = node*2;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 6;   %����ϵͳ����ı���
sizes.NumInputs      = 6;   %����ϵͳ����ı���
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
qd1 = 0.1*sin(t);
qd2 = 0.1*sin(t);
dqd1 = 0.1*cos(t);
dqd2 = 0.1*cos(t);
  
q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;

% �����Ķ���
kv = [10 0; 0 10];
ita = [15 0;0 15];
xite1 = 5;
xite2 = 5;

% ��ģ����
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;
s = [s1;s2];

Input1 = [e1;de1];
hw = zeros(node, 1);   %7*1����
for j = 1:node
    hw(j) = exp(-(norm(Input1 - c(:,j))^2) / (b(j)^2));
end

Input2 = [e2;de2];
hv = zeros(node, 1);   %7*1����
for j = 1:node
    hv(j) = exp(-(norm(Input2 - c(:,j))^2) / (b(j)^2));
end

W = x(1:node);     % node*1
V = x(node+1:2*node);

% WȨֵ�ĸ���
dw1 = ita(1)*hw*s(1);
dw2 = ita(4)*hv*s(2);
for i = 1:node
    sys(i) = dw1(i);
    sys(i+node) = dw2(i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node
% �Ƕȸ���ָ��
qd1 = 0.1*sin(t);
qd2 = 0.1*sin(t);
dqd1 = 0.1*cos(t);
dqd2 = 0.1*cos(t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);
K1 = u(5);
K2 = u(6);

e1 = qd1 - q1;      % e = qd - q
e2 = qd2 - q2;
de1 = dqd1 - dq1;
de2 = dqd2 - dq2;

% �����Ķ���
kv = [20 0; 0 20];
ita = [15 0;0 15];
xite1 = 5;
xite2 = 5;
epc = 2;
bd = 2.1;

% ��ģ����
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;

d1 = norm(s1)/sqrt(xite1^2+1);
d2 = norm(s2)/sqrt(xite2^2+1);

% ��ģ����
s1 = de1 + xite1*e1;
s2 = de2 + xite2*e2;
s = [s1;s2];

Input1 = [e1;de1];
hw = zeros(node, 1);   %7*1����
for j = 1:node
    hw(j) = exp(-(norm(Input1 - c(:,j))^2) / (b(j)^2));
end

Input2 = [e2;de2];
hv = zeros(node, 1);   %7*1����
for j = 1:node
    hv(j) = exp(-(norm(Input2 - c(:,j))^2) / (b(j)^2));
end

W = x(1:node);     % node*1
V = x(node+1:2*node);

% ����������
fx1 = W' * hw;
fx2 = V' * hv;
v = -(epc+bd)*sign(s);

temp1 = (abs(K1)+1)*sign(s1);
temp2 = (abs(K2)+1)*sign(s2);
v_new = -[temp1; temp2];

tau = [fx1;fx2]+kv*s-v;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx1;
sys(4) = fx2;
sys(5) = s1;
sys(6) = s2;











