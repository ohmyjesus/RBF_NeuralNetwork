function [sys,x0,str,ts] = B1(t,x,u,flag)
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
global c b kv kp
sizes = simsizes;
sizes.NumContStates  = 10;   %����ϵͳ����״̬�ı��� W V
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 6;   %����ϵͳ����ı���
sizes.NumInputs      = 10;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 1;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0.1 * ones(1, 10);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [0 0];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
% ���������4 - 5 - 2�ṹ
c = 1 * [-2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2;
         -2 -1  -0 1 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  4*5
b = 3;  % ��˹�����Ļ���  ά��node * 1  5*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
alph = 3;
kp = [alph^2 0; 0 alph^2];
kv = [2 * alph 0;0 2*alph];

function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b kv kp
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
node = 5;
Q = 50 * eye(4);
A = [zeros(2,2) eye(2,2); -kp -kv];     % 4*4
P = lyap(A' , Q);                       % 4*4
B = [0 0;0 0;1 0;0 1];

qd1 = u(1);
qd2 = u(2);  
d_qd1 = u(3);
d_qd2 = u(4);
q1 = u(5);
q2 = u(6);
dq1 = u(7);
dq2 = u(8);

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - d_qd1;
de2 = dq2 - d_qd2;

W = [x(1)  x(2)  x(3)  x(4)  x(5); x(6)  x(7)  x(8)  x(9)  x(10)]';
Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
gama = 20;
dw = gama * h * Input' * P * B;
dw = dw';
for i = 1:1:5
    sys(i) = dw(1,i);
    sys(i+5) = dw(2,i);
end

% �����Ķ���
% v = 13.33;
% a1 = 8.98;
% a2 = 8.75;
% g = 9.8;
% 
% M = [v+a1+2*a2*cos(q2)  a1+a2*cos(q2);
%      a1+a2*cos(q2)  a1];
% Q = 50 * eye(4);
% 
% 
% k1 = 0.001;
% method = 1;
% 
% % WȨֵ�ĸ���
% if method == 1      % ����Ӧ����һ
%     dw = gama * h * Input' * P * B;
%     for i = 1:node*2
%         sys(i) = dw(i);
%     end
% else                % ����Ӧ������
%     dw = gama * h * Input' * P * B + k1 * gama * norm(Input) * [W V];
%     for i = 1:node*2
%         sys(i) = dw(i);
%     end
% end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b kv kp
node = 5;
% �Ƕȸ���ָ��
qd1 = u(1);
qd2 = u(2);  
d_qd1 = u(3);
d_qd2 = u(4);
q1 = u(5);
q2 = u(6);
dq1 = u(7);
dq2 = u(8);
ddq1 = u(9);
ddq2 = u(10);
dd_qd1 = -0.1*pi*0.5*pi*sin(0.5*pi*t);
dd_qd2 = 0.1*pi*0.5*pi*cos(0.5*pi*t);
ddq = [ddq1; ddq2];
dd_qd = [dd_qd1; dd_qd2];

e1 = q1 - qd1;      % e = q - qd
e2 = q2 - qd2;
de1 = dq1 - d_qd1;
de2 = dq2 - d_qd2;
e = [e1; e2];
de = [de1; de2];

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

dq = [dq1; dq2];

tol1 = M*(dd_qd - kv*de - kp*e) + C * dq + G;

deltam = 0.2*M;
deltac = 0.2*C;
deltag = 0.2*G;

d1 = 2;
d2 = 3;
d3 = 6;
d = d1 + d2 * norm([e1;e2]) + d3 * norm([de1; de2]);
f = inv(M) * (deltam * ddq + deltac * dq + deltag + d);

Input = [e1; e2; de1; de2];
h = zeros(node , 1);   %5*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end

W = [x(1)  x(2)  x(3)  x(4)  x(5); x(6)  x(7)  x(8)  x(9)  x(10)]';
fn = W' * h;
tol2 = -M * fn;
tol = tol1 + tol2;

sys(1) = tol(1);
sys(2) = tol(2);
sys(3) = f(1);
sys(4) = fn(1);
sys(5) = f(2);
sys(6) = fn(2);












