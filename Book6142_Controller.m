function [sys,x0,str,ts] = Book6142_Controller(t,x,u,flag)
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
% ���������4 - 5 - 2�ṹ
node = 5;
c = 1 * [-2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  4*5
b = 3 * ones(node,1);  % ��˹�����Ļ���  ά��node * 1  5*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
sizes = simsizes;
sizes.NumContStates  = node*2;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 6;   %����ϵͳ����ı���
sizes.NumInputs      = 8;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0.1 * ones(node*2,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b node
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
% �Ƕȸ���ָ��
dqd1 = 0.1*pi*cos(0.5*pi*t);
dqd2 = 0.1*pi*sin(0.5*pi*t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;

% �����Ķ���
v = 13.33;
a1 = 8.98;
a2 = 8.75;
g = 9.8;
gama = 20;
M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
     a1+a2*cos(q2)  a1];
alph = 3;
kp = [alph^2 0; 0 alph^2];
kv = [2 * alph 0;0 2*alph];
Q = 50 * eye(4);

A = [zeros(2,2) eye(2,2); -kp -kv];     % 4*4
P = lyap(A' , Q);                       % 4*4
% B = [zeros(2,2); inv(M)];                        % 4*2
B = [0 0;0 0;1 0;0 1];
k1 = 0.001;

Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = x(1:5);     % 5*1
V = x(6:10);    

method = 1;

% WȨֵ�ĸ���
if method == 1      % ����Ӧ����һ
    dw = gama * h * Input' * P * B;
    for i = 1:node*2
        sys(i) = dw(i);
    end
else                % ����Ӧ������
    dw = gama * h * Input' * P * B + k1 * gama * norm(Input) * [W V];
    for i = 1:node*2
        sys(i) = dw(i);
    end
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node
% �Ƕȸ���ָ��
ddqd1 = -0.1*pi*0.5*pi*sin(0.5*pi*t);
ddqd2 = 0.1*pi*0.5*pi*cos(0.5*pi*t);

qd1 = u(1);
qd2 = u(2);      
q1 = u(3);
q2 = u(4);
dq1 = u(5);
dq2 = u(6);
ddq1 = u(7);
ddq2 = u(8);
dqd1 = 0.1*pi*cos(0.5*pi*t);
dqd2 = 0.1*pi*sin(0.5*pi*t);
ddq = [ddq1; ddq2];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - dqd1;
de2 = dq2 - dqd2;

% �����Ķ���
v = 13.33;
a1 = 8.98;
a2 = 8.75;
g = 9.8;
d1 = 2;
d2 = 3;
d3 = 6;
alph = 3;
kp = [alph^2 0; 0 alph^2];
kv = [2 * alph 0;0 2*alph];

M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
     a1+a2*cos(q2)  a1];
C = [-a2*dq2*sin(q2)   -a2*(dq1 + dq2)*sin(q2);
     a2*dq1*sin(q2) 0];
G = [15*g*cos(q1)+8.75*g*cos(q1+q2);
     8.75*g*cos(q1+q2)];
deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;

Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = x(1:5);     % 5*1
V = x(6:10);   

% ����������
fx1 = W' * h;
fx2 = V' * h;

% ����
d = d1 + d2 * norm([e1,e2]) + d3 * norm([de1, de2]);
ddqd = [ddqd1; ddqd2];
e = [e1; e2];
de = [de1; de2];
q = [q1; q2];
dq = [dq1; dq2];
f = deltam * ddq + deltac * dq + deltag + d;

some = 1;
if some == 1
    tau = M * (ddqd - kv*de - kp*e) + C * dq + G - M * [fx1; fx2];   % RBF�ƽ�δ֪����f
elseif some == 2
    tau = M * (ddqd - kv*de - kp*e) + C * dq + G - f;            % ��ȷ����������
else
    tau = M * (ddqd - kv*de - kp*e) + C * dq + G ;               % ������������ 
end

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx1;
sys(4) = fx2;
sys(5) = f(1);
sys(6) = f(2);












