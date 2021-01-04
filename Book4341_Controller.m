function [sys,x0,str,ts] = Book4341_Controller(t,x,u,flag)
% ���³����� ����RBF�������ֱ��³������Ӧ����
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}
    sys=[];
  case 3
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %ϵͳ�ĳ�ʼ��
sizes = simsizes;
sizes.NumContStates  = 0;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 1;   %����ϵͳ����ı���
sizes.NumInputs      = 4;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [];                   % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
global W 
% ���������5-9-1�ṹ  IN = 5 MID = 9  OUT = 1
% ��ʼȨֵ
W = [0 0 0 0 0 0 0 0 0]' ;  %MID * OUT����  9*1

    
function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W 
% ���������5-9-1�ṹ
b = 20;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
c = [-2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2;
    -2 -1.5  -1 -0.5 0 0.5 1 1.5 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  5*9
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 5;
Mid = 9;
Out = 1;

lambda = 5;
ita = 500 * eye(9);
xite = 0.005;
If = 0.25;

e = -u(1);       % e = x - xd; ʵ��-����
de = -u(2);
x_1 = u(3);
x_2 = u(4);
s = lambda * e + de;
s_if = s/If;

yd = sin(t);
dyd = cos(t);
ddyd = -sin(t);

v = -ddyd + lambda * de;

Input = [x_1; x_2; s; s_if ; v];
h = zeros(Mid , 1);   %9*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
% ������ut
belta = 150;
ut = 1/belta * W' * h;
sys(1) = ut;

% WȨֵ�ĸ���
dw = -ita * (h * s + xite * W); % 9*9 * (9*1  +  9*1)
dt = 0.001;     % ���沽��
W = W + dw * dt;    % W������Ӧ��







