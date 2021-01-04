function [sys,x0,str,ts] = Book6331_Controller(t,x,u,flag)
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
% ���������2 - 5 - 1�ṹ  
node = 5;
c = 1 * [-1.0 -0.5 0 0.5 1 ;
         -1.0 -0.5 0 0.5 1];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*5
b = 50 * ones(node,1);  % ��˹�����Ļ���  ά��node * 1  7*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
sizes = simsizes;
sizes.NumContStates  = node;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 2;   %����ϵͳ����ı���
sizes.NumInputs      = 4;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0 * ones(node,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b node
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
% �Ƕȸ���ָ��
% qd = sin(t);
dqd =  cos(t);

qd = u(1);
q = u(2);
dq = u(3);

e = q - qd;      % e = q - qd
de = dq - dqd;

% �����Ķ���
xite = 1000;
alpha = 200;
gama = 0.1;

input = [e; de];
h = zeros(node , 1);   %7*1����
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (2*b(i)^2)); % 7*1
end

W = x(1:node);     % 5*1 
% Ȩֵ������Ӧ��
x_2 = de + alpha * e;
dw = - xite * x_2 * h';
for i = 1:node
    sys(i) = dw(i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node
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

e = q - qd;      % e = q - qd
de = dq - dqd;

% �����Ķ���
M = 1;
xite = 1000;
alpha = 200;
gama = 0.1;

input = [e; de];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(input - c(:,i))^2) / (2*b(i)^2)); % 5*1
end
W = x(1:node);     % 5*1 

% ����������
fx = W' * h;
V = 0;
omiga = M * alpha * de + V * alpha * e;
x_2 = de + alpha * e;
ut = -omiga - 1/(2*gama*gama)*x_2 + W' * h - 1/2*x_2;
G = 0;
tau = ut + M*ddqd + V * dqd + G;

sys(1) = tau;
sys(2) = fx;













