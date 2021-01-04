function [sys,x0,str,ts] = Book4342_Controller(t,x,u,flag)
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
global c b node If lamda W
W = [0 0 0 0 0 0 0 0 0 0 0 0 0]' ;  %MID * OUT ����  13*1
node = 13;
If = 0.25;
lamda = 5;
sizes = simsizes;
sizes.NumContStates  = node;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 1;   %����ϵͳ����ı���
sizes.NumInputs      = 3;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = zeros(1,13);            % ϵͳ��ʼ״̬����
% ���������5-9-1�ṹ
c = [-6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6;
    -6 -5  -4 -3 -2 -1 0  1 2 3 4 5 6];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  5*13
b = 5;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys = mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
global c b node If lamda W
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 5;
Mid = 13;
Out = 1;
yd = pi/6 * sin(t);
dyd = pi/6 * cos(t);
ddyd = -pi/6 * sin(t);

% e = -u(1);       % e = x - xd; ʵ��-����
% de = -u(2);
% x_1 = u(3);
% x_2 = u(4);
x1 = u(2);
x2 = u(3);
e = x1 - yd;
de = x2 - dyd;
s = lamda * e + de;  
s_if = s/If;

v = -ddyd + lamda * de;
Input = [x1; x2; s; s_if ; v];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %13*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end

rou = 0.005;
Gama = 15 * eye(13);
W = [x(1); x(2); x(3); x(4); x(5); x(6); x(7); x(8); x(9); x(10); x(11); x(12); x(13)];
S = -Gama * (h*s + rou*W);

for i = 1:node
    sys(i) = S(i);
end


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global c b node If lamda W
yd = pi/6 * sin(t);
dyd = pi/6 * cos(t);
ddyd = -pi/6 * sin(t);
x_1 = u(2);
x_2 = u(3);
e = x_1 - yd;   % e = x - xd; ʵ��-����
de = x_2 - dyd;
s = lamda * e + de;  
s_if = s/If;

v = -ddyd + lamda * de;
Input = [x_1; x_2; s; s_if ; v];
% Input = [x_1; x_2; s; s_if ; v];
h = zeros(node , 1);   %13*1����
for i =1:node
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
W = [x(1); x(2); x(3); x(4); x(5); x(6); x(7); x(8); x(9); x(10); x(11); x(12); x(13)];
belta = 1;
ut = 1/belta * W' * h;
sys(1) = ut;












