function [sys,x0,str,ts] = Book523_Controller(t,x,u,flag)
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
sizes = simsizes;
sizes.NumContStates  = 5;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 2;   %����ϵͳ����ı���
sizes.NumInputs      = 3;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = zeros(1,5);            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
global c b 
% ���������2-5-1�ṹ
c = 0.1*[-1 -0.5  -0 0.5 1;
     -1 -0.5  -0 0.5 1];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*5
b = 5;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b gama 
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 2;
Mid = 5;
Out = 1;
yd = 0.1 * sin(t);
dyd = 0.1 * cos(t);
ddyd = -0.1 * sin(t);

c1 = 15;
gama = 0.015;
e = u(1);
de = u(2);

s = c1 * e + de;  

Input = [e; de];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %5*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
S = -1/gama * s  * h;

for i = 1:Mid
    sys(i) = S(i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b 
IN = 2;
Mid = 5;
Out = 1;
yd = 0.1 * sin(t);
dyd = 0.1 * cos(t);
ddyd = -0.1 * sin(t);
c1 = 15;

e = u(1);
de = u(2);
th = u(3);

s = c1 * e + de;  

Input = [e; de];

h = zeros(Mid , 1);   %13*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
fx = W' * h;

% �����Ķ���
mc = 1;     %С������
m = 0.1;    %�ڵ�����
l = 0.5;
g_up = cos(th)/(mc+m);
g_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
g = g_up / g_down;
if t<=1.5
    xite = 1.0;
else
    xite = 0.1;
end
ut = 1 / g * (-fx + ddyd + c1 * de + xite * sign(s));
sys(1) = ut;
sys(2) = fx;












