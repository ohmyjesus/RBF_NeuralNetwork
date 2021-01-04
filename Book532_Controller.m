function [sys,x0,str,ts] = Book532_Controller(t,x,u,flag)
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
sizes.NumContStates  = 10;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 4;   %����ϵͳ����ı���
sizes.NumInputs      = 4;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = 0.1 * ones(10,1);            % ϵͳ��ʼ״̬���� ����W��V���� 
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
global c b 
% ���������2-5-1�ṹ
c = 2*[-1 -0.5  -0 0.5 1;
     -1 -0.5  -0 0.5 1];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*5
b = 10;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b 
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 2;
Mid = 5;
Out = 1;
yd = 0.1 * sin(t);
dyd = 0.1 * cos(t);
ddyd = -0.1 * sin(t);

c1 = 5;
gama1 = 10;
gama2 = 10;
xite = 0.01;
x_1 = u(1);
x_2 = u(2);
e = u(3);
de = u(4);

s = c1 * e + de;  

Input = [x_1; x_2];
h = zeros(Mid , 1);   %5*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
fx = W' * h;
S1 = -gama1 * s * h;
for i = 1:5
    sys(i) = S1(i);
end
th = u(1);      % �ڽ�
dth = u(2);     % ����

% �����Ķ���
g = 9.8;
mc = 1;     %С������
m = 0.1;    %�ڵ�����
l = 0.5;
f_up = g*sin(th) - m*l*dth^2*cos(th)*sin(th)/(mc+m);
f_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
f = f_up / f_down;

V = [x(6); x(7); x(8); x(9); x(10)];
gx = V' * h ;
ut = 1 / gx * (-f + ddyd + c1 * de + xite * sign(s));
S2 = -gama2 * s * h * ut;
for i = 6:10
    sys(i) = S2(i - 5);
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

x_1 = u(1);
x_2 = u(2);
e = u(3);
de = u(4);

s = c1 * e + de;  

Input = [x_1; x_2];

h = zeros(Mid , 1);   %13*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5)];
fx = W' * h;

V = [x(6); x(7); x(8); x(9); x(10)];
gx = V' * h ;

th = x(1);      % �ڽ�
dth = x(2);     % ����

% �����Ķ���
g = 9.8;
mc = 1;     %С������
m = 0.1;    %�ڵ�����
l = 0.5;
f_up = g*sin(th) - m*l*dth^2*cos(th)*sin(th)/(mc+m);
f_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
f = f_up / f_down;
g_up = cos(th)/(mc+m);
g_down = l *(4/3 - m*(cos(th)^2)/(mc+m));
g = g_up / g_down;

% if t<=1.5
%     xite = 1.0;
% else
%     xite = 0.1;
% end

xite = 1;
ut = 1 / gx * (-f + ddyd + c1 * de + xite * sign(s));

sys(1) = ut;
sys(2) = fx;
sys(3) = gx;
sys(4) = s;












