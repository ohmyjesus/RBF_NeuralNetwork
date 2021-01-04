function [sys,x0,str,ts] = Book7241_Controller(t,x,u,flag)
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
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
% ���������2 - 7 - 1�ṹ  ����3��2*7*1 ������
node = 7;
c1 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*7
b1 = 20 * ones(node,1);                             % ��˹�����Ļ���  ά��node * 1  7*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ�� 
c2 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];  
b2 = 20 * ones(node,1);
c3 = 1 * [-1.5 -1.0 -0.5 0 0.5 1 1.5;
         -1.5 -1.0 -0.5 0 0.5 1 1.5];  
b3 = 20 * ones(node,1);
s_new = 0;
s_past = s_new;
inte_s = 0;
sizes = simsizes;
sizes.NumContStates  = node*3;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 4;   %����ϵͳ����ı���
sizes.NumInputs      = 4;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0 * ones(node*3,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
% �Ƕȸ���ָ��
% qd = sin(t);
dqd =  cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);
dq = u(3);
ddq = u(4);

e = qd - q;      % e = qd - q
de = dqd - dq;
dde = ddqd - ddq;

% �����Ķ���
kr = 0.1;
kp = 15;
kt = 15;
xite = 5.0;
gamam = 100;
gamac = 100;
gamag = 100;

s_new = xite * e + de;
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% �����������
input = [q; dq];
h1 = zeros(node , 1);   %7*1����
h2 = zeros(node , 1);   %7*1����
h3 = zeros(node , 1);   %7*1����
for i =1:node
    h1(i) = exp(-(norm(input - c1(:,i))^2) / (b1(i)^2)); % 7*1
end
for i =1:node
    h2(i) = exp(-(norm(input - c2(:,i))^2) / (b2(i)^2)); % 7*1
end
for i =1:node
    h3(i) = exp(-(norm(input - c3(:,i))^2) / (b3(i)^2)); % 7*1
end

W1 = x(1:node);     % 7*1
W2 = x(node+1: node*2);
W3 = x(node*2+1: node*3);

% Ȩֵ������Ӧ��
dw1 = gamam * h1 * ddqr * s_new;
dw2 = gamac * h2 * dqr * s_new;
dw3 = gamag * h3 * s_new;

for i = 1:node
    sys(i) = dw1(i);
    sys(i+7) = dw2(i);
    sys(i+14) = dw3(i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c1 b1 node c2 b2 c3 b3 s_new s_past inte_s
% �Ƕȸ���ָ��
% dqd1 =  0.1*cos(t);
% dqd2 =  0.1*cos(t);
% qd = sin(t);
dqd =  cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);
dq = u(3);
ddq = u(4);

e = qd - q;      % e = qd - q
de = dqd - dq;
dde = ddqd - ddq;

% �����Ķ���
kr = 0.1;
kp = 15;
ki = 15;
xite = 5.0;
gamam = 100;
gamac = 100;
gamag = 100;

s_new = xite * e + de;
dqr = xite * e + dqd;
ddqr = xite * de + ddqd;

% �����������
input = [q; dq];
h1 = zeros(node , 1);   %7*1����
h2 = zeros(node , 1);   %7*1����
h3 = zeros(node , 1);   %7*1����
for i =1:node
    h1(i) = exp(-(norm(input - c1(:,i))^2) / (b1(i)^2)); % 7*1
end
for i =1:node
    h2(i) = exp(-(norm(input - c2(:,i))^2) / (b2(i)^2)); % 7*1
end
for i =1:node
    h3(i) = exp(-(norm(input - c3(:,i))^2) / (b3(i)^2)); % 7*1
end

W1 = x(1:node);     % 7*1
W2 = x(node+1: node*2);
W3 = x(node*2+1: node*3);

% ����������
fx1 = W1' * h1;
fx2 = W2' * h2;
fx3 = W3' * h3;

M_refer = fx1;
C_refer = fx2;
G_refer = fx3;

% ����ģ�Ϳ�����
taum = M_refer*ddqr + C_refer*dqr + G_refer;
% ³����
taur = kr*sign(s_new);
dt = 0.001;
inte_s = inte_s + (s_past + s_new)*dt/2;
tau = taum + kp*s_new + ki*inte_s + taur;


sys(1) = tau;
sys(2) = fx1;
sys(3) = fx2;
sys(4) = fx3;
s_past = s_new;












