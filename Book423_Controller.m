function [sys,x0,str,ts] = Book423_Controller(t,x,u,flag)
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
sizes.NumOutputs     = 2;   %����ϵͳ����ı���
sizes.NumInputs      = 2;   %����ϵͳ����ı���
sizes.DirFeedthrough = 1;   %���������������Ժ��������u����Ӧ�ý�����������Ϊ1
sizes.NumSampleTimes = 0;   % ģ��������ڵĸ���
                            % ��Ҫ������ʱ�䣬һ��Ϊ1.
                            % �²�Ϊ���Ϊn������һʱ�̵�״̬��Ҫ֪��ǰn��״̬��ϵͳ״̬
sys = simsizes(sizes);
x0  = [];                   % ϵͳ��ʼ״̬����
str = [];                   % ��������������Ϊ��
ts  = [];                   % ����ʱ��[t1 t2] t1Ϊ�������ڣ����ȡt1=-1�򽫼̳������źŵĲ������ڣ�����t2Ϊƫ������һ��ȡΪ0
global W m_0
% ���������2-5-1�ṹ  IN = 2 MID = 5  OUT = 1
% ��ʼȨֵ
W = [0 ; 0 ; 0 ; 0  ; 0]';  %MID * OUT����  1*5
m_0 = 120;
    
function sys = mdlOutputs(t,x,u)   %���������ݣ�ϵͳ���
global W m_0
% ���������2-5-1�ṹ
b = 100;  % ��˹�����Ļ���  ά��MID * 1  1*1   b��ѡ�����Ҫ bԽ�� ��·�������ӳ������Խ��  
c = [-2 -1 0 1 2;
    -2 -1 0 1 2];               % ��˹���������ĵ�ʸ�� ά�� IN * MID  2*5
% ������Ӧ������������ֵ����Чӳ�䷶Χ����� c��b �Ӷ���֤��Ч�ĸ�˹ӳ��  �����ʵ�b��c���ᵼ�½������ȷ
IN = 2;
Mid = 5;
Out = 1;

Q = [500 0; 0 500];
kd = 50;
kp = 30;
gama = 1200;         %gamaΪ������
xite = 0.0001;
m = 100;

e = u(1);
de = u(2);

Input = [u(1); u(2)];
h = zeros(Mid , 1);   %5*1����
for i =1:Mid
    h(i) = exp(-(norm(Input - c(:,i))^2) / (2*b^2));
end
% fx�Ĺ���
fx_refer = W * h;

K = [kp ;kd];
E = [e ; de];
yd = sin(t);
dyd = cos(t);
ddyd = -sin(t);

% ������ut
ut = 1/(m_0)*(-fx_refer + ddyd + K' * E);

sys(1) = ut;
sys(2) = fx_refer;

% ����Ӧ�ɵ����
fai = [0 1; -kp -kd];
P = lyap(fai', Q);   %PΪ�Գ���������������Lyapunov����
B = [0; 1];
dw = zeros(1, 5);
for i = 1:5
    dw(i) = -gama * E' * P  * B * h(i);    % 1*1 * 1*2 * 2*2 *2*1 * 1*1 
end
dt = 0.001;     % ���沽��
W = W + dw * dt;    % W������Ӧ��

% m�Ĺ�����
some = E' * P * B * ut;
if some > 0
    dm = 1/xite*some;
elseif some <= 0 && m_0 > m
    dm = 1/xite*some;
else
    dm = 1/xite;
end
m_0 = m_0 + dm * dt;    % m������Ӧ��





