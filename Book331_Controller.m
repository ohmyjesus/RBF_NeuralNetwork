function [sys,x0,str,ts] = Book331_Controller(t,x,u,flag)
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
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [];                   % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
global W_past W_new V_new V_past c b ut_new ut_past
% ���������1-6-1�ṹ
W_new = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5];  %6*1����
W_past = W_new;
V_new = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5];  %6*1����
V_past = V_new;
c = [0.5  0.5  0.5  0.5  0.5  0.5];
b = [5;5;5;5;5;5];
ut_new = 0;
ut_past = ut_new;
    
function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W_past W_new V_new V_past c b ut_new ut_past
% ���������1-6-1�ṹ
xitew = 0.15;   %ѧϰ����1
xitev = 0.5;    %ѧϰ����2
alfa = 0.05;   %��������
IN = 1;
Mid = 6;
Out = 1;
yd = u(1);
yk = u(2);

some = [u(1); u(2)];
h = zeros(1,6);   %1*6����
for i =1:Mid
    h(i) = exp(-(norm(yk - c(:,i))^2) / (2*b(i)^2));
end
ut1 = h * W_new;
ut2 = h * V_new;

ut_new = -(ut1/ut2) + (yd/ut2);
sys(1) = ut_new;

% Ȩֵ������
deltaw = zeros(6,1);
for i = 1:6
    deltaw(i) = xitew * (yk - yd) * h(i);
end
W_new = W_new + deltaw + alfa*(W_new - W_past);

deltav = zeros(6,1);
for i = 1:6
    deltav(i) = xitev * (yk - yd) * h(i) * ut_new;
end
V_new = V_new + deltav + alfa*(V_new - V_past);

W_past = W_new;
V_past = V_new;
ut_past = ut_new;






