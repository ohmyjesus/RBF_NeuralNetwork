function [sys,x0,str,ts] = Book2222_controller(t,x,u,flag)
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
global  W_new W_past C_new C_past  B_new B_past 
C_new = [-1  -0.5  0  0.5  1;  
        -10  -5  0  5  10];   %2*5  ����ʸ��
C_past = C_new;
B_new = [3  3  3  3  3];  %1*5  ����Ȳ���
B_past = B_new;
W_new = rand(1,5);  %Ȩֵȡ0-1�����ֵ
W_past = W_new;


function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W_new W_past C_new C_past  B_new B_past 
alpha = 0.05; %����ϵ��
xite = 0.15;   %ѧϰЧ��
u_in = u(1);
y_out = u(2);  
some = [u_in; y_out];
h = zeros(5,1);
for j = 1:5
    h(j) = exp(-(norm(some - C_new(:,j))^2/(2* B_new(j)^2)));  %5*1����  ���������
end
% RBF���������ym
ym = W_new * h;

% Ȩֵ�ĵ��� ����ֵ
deltaW = zeros(1,5);
for i = 1:5
    deltaW(i) = xite * (y_out - ym) * h(i);
end
for i = 1:5
    W_new(i) = W_new(i) + deltaW(i) + alpha*(W_new(i) - W_past(i));
end

% ���������b������
deltab = zeros(1,5);
for i = 1:5
    deltab(i) = xite * (y_out - ym) * W_new(i) * h(i) * (norm(some - C_new(:,i))^2 / B_new(i)^3);
    B_new(i) = B_new(i) + deltab(i) + alpha*(B_new(i) - B_past(i));
end

% ����ʸ��c������
deltac = zeros(2,5);
for j = 1:2
    for i = 1:5 
        deltac(j,i) = xite * (y_out - ym) * W_new(i) * h(i) * ((some(j) - C_new(j,i)) / B_new(i)^2);
        C_new(j,i) = C_new(j,i) + deltac(j,i) + alpha*(C_new(j,i) - C_past(j,i));
    end
end

sys(1) = ym;

W_past = W_new;
B_past = B_new;
C_past = C_new;



