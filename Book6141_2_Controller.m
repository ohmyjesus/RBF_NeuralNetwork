function [sys,x0,str,ts] = Book6141_2_Controller(t,x,u,flag)
% ���³����� ����RBF�������ֱ��³������Ӧ����
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
% ���������2-19-1�ṹ
node = 19;
c = 0.5 * [-4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5 3 3.5 4 4.5;
           -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.5 3 3.5 4 4.5];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*19
b = 2 * ones(19,1);  % ��˹�����Ļ���  ά��node * 1  19*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
sizes = simsizes;
sizes.NumContStates  = node;   %����ϵͳ����״̬�ı��� W
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 3;   %����ϵͳ����ı���
sizes.NumInputs      = 3;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0.1 * ones(node,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b node
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 2;
Out = 1;
qd = sin(t);
dqd = cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);      % e = q - qd
dq = u(3);
e = q - qd;
de = dq - dqd;

% �����Ķ���
M = 10;
gama = 1200;
alph = 3;
kp = alph^2;
kv = 2 * alph;
Q = [50 0; 0 50];
A = [0 1; -kp -kv];
P = lyap(A' , Q);
B = [0; 1/M];
k1 = 0.001;

Input = [e; de];
h = zeros(node , 1);   %19*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = [x(1:19)]';     %node*1  19*1

method = 1;
if method == 1      % ����Ӧ����һ
    dw = gama * h * Input' * P * B;
    for i = 1:node
    sys(i) = dw(i);
    end
else                % ����Ӧ������
    dw = gama * h * Input' * P * B + k1 * gama * norm(Input) * W;
    for i = 1:node
    sys(i) = dw(i);
    end
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node
IN = 2;
Out = 1;
qd = sin(t);
dqd = cos(t);
ddqd = -sin(t);

qd = u(1);
q = u(2);      % e = q - qd
dq = u(3);
e = q - qd;
de = dq - dqd;

% �����Ķ���
M = 10;
gama = 1200;
alph = 3;
kp = alph^2;
kv = 2 * alph;
Q = [50 0; 0 50];
A = [0 1; -kp -kv];
P = lyap(A' , Q);
B = [0; 1/M];
k1 = 0.001;

Input = [e; de];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b(i)^2));
end
W = [x(1:19)];     %node*1  19*1
% ����������
fx = W' * h;
d = -15 * dq - 30 *sign(dq);

some = 1;
if some == 1
    ut = M * (ddqd - kv*de - kp*e) - fx;
elseif some == 2
    ut = M * (ddqd - kv*de - kp*e) - d;
else
    ut = M * (ddqd - kv*de - kp*e);
end
sys(1) = ut;
sys(2) = fx;
sys(3) = d;












