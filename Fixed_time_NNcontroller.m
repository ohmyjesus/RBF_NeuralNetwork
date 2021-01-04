function [sys,x0,str,ts] = Fixed_time_NNcontroller(t,x,u,flag)
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
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% ���������2-7-1�ṹ
node = 7;
c1 = 0.5*[-2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*7
b1 = 20;  % ��˹�����Ļ���  ά��MID * 1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
gama1 = 10;
lg = 2;
xite = 20;   % �������Ĳ���

c2 = 1.8*[-2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2;
          -2 -1.5 -1 0 1 1.5 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*7
b2 = 4;  % ��˹�����Ļ���  ά��MID * 1    b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ�� 

gama2 = 0.01;   
belta = 2;
sizes = simsizes;
sizes.NumContStates  = node*6;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 4;   %����ϵͳ����ı���
sizes.NumInputs      = 7;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 1;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0.1*[ones(node*4,1); 2*ones(node*1,1);3*ones(node*1,1)];            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [0 0];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
yd1 = 0.5*sin(pi*t);
dyd1 = 0.5*pi*cos(pi*t);
ddyd1 = -0.5*pi*pi*sin(pi*t);

yd2 = 0.5*sin(pi*t);
dyd2 = 0.5*pi*cos(pi*t);
ddyd2 = -0.5*pi*pi*sin(pi*t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = yd1 - q1;
e2 = yd2 - q2;
de1 = dyd1 - dq1;
de2 = dyd2 - dq2;
e = [e1;e2];
de = [de1;de2];

%  �����Ķ���
q = 3;
p = 5;

% ��ģ��
s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
s = [s1; s2];

coe1 = p/(belta*q)*de1^(p-q)^(1/q);    % �ش���0
coe2 = p/(belta*q)*de2^(p-q)^(1/q);    % �ش���0

Input = [q1;q2;dq1;dq2];
% --------------------------------------------- WȨֵ�ĸ���
hf1 = zeros(node , 1);   %7*1����
hf2 = zeros(node , 1);   %7*1����
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
end
W1 = x(1:node);         % 7*1����
W2 = x(node+1:2*node);  % 7*1����

fx1 = W1' * hf1;
fx2 = W2' * hf2;

fx = [fx1; fx2];           % 2*1
dw_fx1 = -gama1 * s1 * coe1 * hf1; % 7*1����
dw_fx2 = -gama1 * s2 * coe2 * hf2; % 7*1����

for i = 1:node
    sys(i) = dw_fx1(i);
    sys(i+node) = dw_fx2(i);
end
% ------------------------------------------------- VȨֵ�ĸ���
hg11 = zeros(node , 1);   %7*1����
hg12 = zeros(node , 1);   %7*1����
hg21 = zeros(node , 1);   %7*1����
hg22 = zeros(node , 1);   %7*1����
for i =1:node
    hg11(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg12(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg21(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg22(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
end
V11 = x(node*2+1:node*3);
V12 = x(node*3+1:node*4);
V21 = x(node*4+1:node*5);
V22 = x(node*5+1:node*6);

gx11 = V11' * hg11;
gx12 = V12' * hg12;
gx21 = V21' * hg21;
gx22 = V22' * hg22;

gx = [gx11  gx12; gx21 gx22];
% �����������
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;

M = [p1+p2+2*p3*cos(q2)  p2+p3*cos(q2);
     p2+p3*cos(q2)  p2];
% �����������
g = inv(M);
temp1 = -belta*q/p* (abs(de1)^(2-p/q)*sign(de1)) + fx(1) - (lg+xite)*sign(s1) - ddyd1;
temp2 = -belta*q/p* (abs(de2)^(2-p/q)*sign(de2)) + fx(2) - (lg+xite)*sign(s2) - ddyd2;
temp = [temp1; temp2];
tau = -inv(gx) * temp;

dw_g11 = -gama2 * s1 * coe1 *  hg11 * tau(1);
dw_g12 = -gama2 * s1 * coe1 *  hg12 * tau(1);

dw_g21 = -gama2 * s2 * coe2 *  hg21 * tau(2);
dw_g22 = -gama2 * s2 * coe2 *  hg22 * tau(2);

for i = 1 : node
    sys(i+node*2) = dw_g11(i);
    sys(i+node*3) = dw_g12(i);
    sys(i+node*4) = dw_g21(i);
    sys(i+node*5) = dw_g22(i);
end

function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c1 b1 c2 b2 node gama1 gama2 xite belta lg
% �Ƕȸ���ָ��
yd1 = 0.5*sin(pi*t);
dyd1 = 0.5*pi*cos(pi*t);
ddyd1 = -0.5*pi*pi*sin(pi*t);

yd2 = 0.5*sin(pi*t);
dyd2 = 0.5*pi*cos(pi*t);
ddyd2 = -0.5*pi*pi*sin(pi*t);

q1 = u(1);
q2 = u(2);
dq1 = u(3);
dq2 = u(4);

e1 = yd1 - q1;
e2 = yd2 - q2;
de1 = dyd1 - dq1;
de2 = dyd2 - dq2;
e = [e1;e2];
de = [de1;de2];
ddyd = [ddyd1; ddyd2];

%  �����Ķ���
q = 3;
p = 5;

% ��ģ��
s1 = e1 + 1/belta*abs(de1)^(p/q)*sign(de1);
s2 = e2 + 1/belta*abs(de2)^(p/q)*sign(de2);
s = [s1; s2];

Input = [q1;q2;dq1;dq2];
% ------------------------------------------- WȨֵ
hf1 = zeros(node , 1);   %7*1����
hf2 = zeros(node , 1);   %7*1����
for i =1:node
    hf1(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
    hf2(i) = exp(-(norm(Input - c1(:,i))^2) / (2*b1^2));
end
W1 = x(1:node);         % 7*1����
W2 = x(node+1:2*node);  % 7*1����

coe1 = p/(belta*q)*de1^(p-q)^(1/q);    % �ش���0
coe2 = p/(belta*q)*de2^(p-q)^(1/q);    % �ش���0

fx1 = W1' * hf1;
fx2 = W2' * hf2;

fx = [fx1; fx2];           % 2*1

% ------------------------------------------- VȨֵ
hg11 = zeros(node , 1);   %7*1����
hg12 = zeros(node , 1);   %7*1����
hg21 = zeros(node , 1);   %7*1����
hg22 = zeros(node , 1);   %7*1����
for i =1:node
    hg11(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg12(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg21(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
    hg22(i) = exp(-(norm(Input - c2(:,i))^2) / (2*b2^2));
end
V11 = x(node*2+1:node*3);
V12 = x(node*3+1:node*4);
V21 = x(node*4+1:node*5);
V22 = x(node*5+1:node*6);

gx11 = V11' * hg11;
gx12 = V12' * hg12;
gx21 = V21' * hg21;
gx22 = V22' * hg22;

gx = [gx11  gx12; gx21 gx22];
p1 = 2.9;
p2 = 0.76;
p3 = 0.87;
p4 = 3.04;

M = [p1+p2+2*p3*cos(q2)  p2+p3*cos(q2);
     p2+p3*cos(q2)  p2];
% �����������
g = inv(M);

temp1 = -belta*q/p* (abs(de1)^(2-p/q)*sign(de1)) + fx(1) - (lg+xite)*sign(s1) - ddyd1;
temp2 = -belta*q/p* (abs(de2)^(2-p/q)*sign(de2)) + fx(2) - (lg+xite)*sign(s2) - ddyd2;
temp = [temp1; temp2];
tau = - inv(gx) * temp;

sys(1) = tau(1);
sys(2) = tau(2);
sys(3) = fx1;
sys(4) = fx2;














