function [sys,x0,str,ts] = Book321_controller(t,x,u,flag)
% RBF����������Ӧ��������������3.2.1����
% ����RBF�������ģ�Ͳο�����Ӧ����
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
sizes.NumInputs      = 3;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1 = -1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
% Ȩֵ��ֵ��ѡ��
% ������PID������ 2-7-1�ṹ
global  W_new W_past C_new C_past  B_new B_past yk_new yk_past ut_new ut_past some
C_new = [-3  -2  -1  0  1  2  3;
        -3  -2  -1  0  1  2  3;
        -3  -2  -1  0  1  2  3];   %3*7  ����ʸ��
C_past = C_new;
B_new = [2  2  2  2  2  2  2];  %1*7  ����Ȳ���
B_past = B_new;
% W_new = rand(1,7);  %Ȩֵȡ0-1�����ֵ
W_new = [-0.0316  -0.0421 -0.0318 0.0068 0.0454 -0.0381 -0.0381];
W_past = W_new;
yk_new = 0;
yk_past = yk_new;
ut_new = 0;
ut_past = ut_new;
some = [0 0 0]';

function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W_new W_past C_new C_past  B_new B_past  yk_new yk_past ut_new ut_past some
% if t>0
% ������PID������ 2-6-1�ṹ
alpha = 0.05; %����ϵ��
xite = 0.35;   %ѧϰЧ��
IN = 3;
Mid = 7;
Out = 1;

yd = u(1);
ec = u(2);  
yk_new = u(3);

h = zeros(Mid, 1);
for j = 1:Mid
    h(j) = exp(-(norm(some - C_new(:,j))^2/(2* B_new(j)^2)));  %7*1����  ���������
end
% RBF���������ym
ut_new = W_new * h;
dyu = sign((yk_new - yk_past)/(ut_new - ut_past));

% Ȩֵ�ĵ��� ����ֵ
deltaW = zeros(1,Mid);
for i = 1:Mid
    deltaW(i) = xite *  ec * dyu * h(i);
end
W_new = W_new + deltaW + alpha*(W_new - W_past);

% ���������b������
% deltab = zeros(1,Mid);
% for i = 1:Mid
%     deltab(i) = xite * dyu * W_new(i) * h(i) * (norm(some - C_new(:,i))^2 / B_new(i)^3);
%     B_new(i) = B_new(i) + deltab(i) + alpha*(B_new(i) - B_past(i));
% end

% ����ʸ��c������
% deltac = zeros(IN, Mid);
% for i = 1:IN
%     for j = 1:Mid 
%         deltac(i,j) = xite * dyu * W_new(j) * h(j) * ((some(i) - C_new(i,j)) / B_new(j)^2);
%         C_new(i,j) = C_new(i,j) + deltac(i,j) + alpha*(C_new(i,j) - C_past(i,j));
%     end
% end

sys(1) = ut_new;
some = [u(1); u(2); u(3)];
W_past = W_new;
B_past = B_new;
C_past = C_new;
ut_past = ut_new;
yk_past = yk_new;
% else
%     sys(1) = ut_new;
% end



