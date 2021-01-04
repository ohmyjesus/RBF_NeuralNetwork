function [sys,x0,str,ts] = Book2221_controller(t,x,u,flag)
% RBF����������Ӧ��������������2.2.2.1����
% �����ݶ��½�����RBF������ƽ�
switch flag
  case 0 %��ʼ��
    [sys,x0,str,ts]=mdlInitializeSizes;
  case {1,2,4,9} %��ɢ״̬���㣬��һ������ʱ�̣���ֹ�����趨
    sys=[];
  case 3 %����źż���
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %ϵͳ�ĳ�ʼ��
sizes = simsizes;
sizes.NumContStates  = 0;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 1;   %����ϵͳ����ı���
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
% Ȩֵ��ֵ��ѡ��
% ������PID������ 2-5-1�ṹ
global  W_new W_past C  B 
C = [-1  -0.5  0  0.5  1;  
        -10  -5  0  5  10];   %2*5  ����ʸ��
B = [1.5  1.5  1.5  1.5  1.5];  %1*5  ����Ȳ���
W_new = rand(1,5);
W_past = W_new;


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W_new W_past C  B 
alpha = 0.05; %����ϵ��
xite = 0.5;   %ѧϰЧ��
u_in = u(1);
y_out = u(2);  
some = [u_in; y_out];
h = zeros(5,1);
for j = 1:5
    h(j) = exp(-norm(some - C(:,j))^2/(2 * B(j)^2));  %6*1����  ���������
end
% RBF���������ym
ym = W_new * h;

% Ȩֵ�ĵ��� ����ֵ
deltaW_new = zeros(1,5);
for i = 1:5
    deltaW_new(i) = xite * (y_out - ym) * h(i);
end

for i = 1:5
    W_new(i) = W_new(i) + deltaW_new(i) + alpha*(W_new(i) - W_past(i));
end

sys(1) = ym;

W_past = W_new;




