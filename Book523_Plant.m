function [sys,x0,str,ts] = Book523_Plant(t,x,u,flag)
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
sizes.NumContStates  = 2;   %����ϵͳ����״̬�ı���
sizes.NumDiscStates  = 0;   %����ϵͳ��ɢ״̬�ı���
sizes.NumOutputs     = 2;   %����ϵͳ����ı���
sizes.NumInputs      = 1;   %����ϵͳ����ı���
sizes.DirFeedthrough = 0;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [pi/60 0];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
% ������״̬����
ut = u(1);     
% f = u(2);
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

some = f + g * ut;

sys(1) = x(2);   
sys(2) = some;   
 

function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
sys(1) = x(1);   %x1
sys(2) = x(2);   %x2




