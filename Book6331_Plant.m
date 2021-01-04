function [sys,x0,str,ts] = Book6331_Plant(t,x,u,flag)
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
sizes.NumOutputs     = 3;   %����ϵͳ����ı���
sizes.NumInputs      = 1;   %����ϵͳ����ı���
sizes.DirFeedthrough = 0;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [0 0];            % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0


function sys=mdlDerivatives(t,x,u)  %�ú�����������ϵͳ�б����ã����ڲ�������ϵͳ״̬�ĵ���
tau = u(1);    %����1

q = x(1);      % �ؽڽ�һ
dq = x(2);     % �ؽڽ��ٶ�һ

% �����Ķ���
M = 1.0;
d = 150 * sign(dq) + 10 * dq;

ddq = inv(M) * (tau - d);

sys(1) = x(2);   
sys(2) = ddq;


function sys=mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
tau = u(1);    %����1

q = x(1);      % �ؽڽ�һ
dq = x(2);     % �ؽڽ��ٶ�һ

% �����Ķ���
M = 1.0;
d = 150 * sign(dq) + 10 * dq;

ddq = inv(M) * (tau - d);
sys(1) = x(1);   %q1
sys(2) = x(2);   %dq1
sys(3) = d;




