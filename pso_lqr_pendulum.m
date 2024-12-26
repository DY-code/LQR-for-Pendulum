% https://fuchaojie.com/2022/08/LQR-for-Cartpole/
clc, clear, close all;
%% ����Ⱥ�Ż� PSO
nPop = 50; % ����Ⱥ��ģ
nVar = 4; % ����ά��
MaxIt = 100; % ����������
w = 1; % ����Ȩ��
c1 = 2; % ѧϰ����1
c2 = 2; % ѧϰ����2
lb = 0.001 * ones(1,nVar); % ��������
ub = 10 * ones(1,nVar); % ��������

% ��ʼ��λ��
pop.Position = lb + (ub - lb).*rand(nPop, nVar);
% ��ʼ���ٶ�
pop.Velocity = zeros(nPop, nVar);

% ������Ӧ��ֵ
for i = 1:nPop
    Q = diag(pop.Position(i,:));
    pop.Fitness(i) = get_fitness(Q);
end
% ��ʼ�����弫ֵ
pop.pBest.Position = pop.Position;
pop.pBest.Fitness = pop.Fitness;
% �ҵ�ȫ�ּ�ֵ
[minFitness,minIndex] = min(pop.Fitness);
pop.gBest.Position = pop.Position(minIndex,:);
pop.gBest.Fitness = minFitness;

fitness_val = [];
for it = 1:MaxIt
    % �����ٶ�
    pop.Velocity = w*pop.Velocity + c1*rand(nPop,nVar).*(pop.pBest.Position - pop.Position) + c2*rand(nPop,nVar).*(repmat(pop.gBest.Position,nPop,1)-pop.Position);
    % ����λ�ò�ȷ���ڷ�Χ��
    pop.Position = pop.Position + pop.Velocity;
    pop.Position = max(pop.Position,lb);
    pop.Position = min(pop.Position,ub);
    % �����µ���Ӧ��ֵ
    for i = 1:nPop
        Q = diag(pop.Position(i,:));
        pop.Fitness(i) = get_fitness(Q);
    end
    % ���¸��弫ֵ
    for i = 1:nPop
        if pop.Fitness(i) < pop.pBest.Fitness(i)
            pop.pBest.Position(i,:) = pop.Position(i,:);
            pop.pBest.Fitness(i) = pop.Fitness(i);
        end
    end
    % ����ȫ�ּ�ֵ
    [minFitness,minIndex] = min(pop.Fitness);
    if minFitness < pop.gBest.Fitness
        pop.gBest.Position = pop.Position(minIndex,:);
        pop.gBest.Fitness = minFitness;
    end
    
    fitness_val = [fitness_val, pop.gBest.Fitness];
    disp(['it: ', num2str(it)]);
    disp(['���Ž�: ', num2str(pop.gBest.Position)]);
    disp(['����ֵ: ', num2str(pop.gBest.Fitness)]);
end

%% ������Ӧ�Ⱥ����仯����
figure;
plot(1:MaxIt, fitness_val, '-*');
xlabel('iter');
ylabel('��Ӧ�Ⱥ���');

%% ��֤��ͼ
mp = 1.0;
mc = 2.0;
l = 0.5;
g = 9.81;

Hl = 1/((mc+mp)*mp*(l^2) - mp^2*l^2) ...
    * [mp*l^2, mp*l; mp*l, mc+mp];
Hl_inv = inv(Hl);
Ac = [0, 0, 1, 0; 0, 0, 0, 1;
    0, Hl_inv(1,2)*mp*g*l, 0, 0;
    0, Hl_inv(2,2)*mp*g*l, 0, 0];
Bc = [0; 0; Hl_inv(1,1); Hl_inv(2,1)];

R = 1;
Q = diag(pop.gBest.Position);

[K,S,P] = lqr(Ac, Bc, Q, R);
K

%%
% ���΢�ַ���
y0 = [0; pi/2; 0; 0];
opt = odeset; opt.RelTol = 1e-8;
[t, x] = ode45(@model, [0,150], y0, opt);

figure;
plot(x, '-');
legend('$x$', '$\theta$', '$\dot{x}$', '$\dot{\theta}$', 'Interpreter', 'latex');
xlabel('step');
ylabel('value');

%% ��ȡ��Ӧ�Ⱥ���
function J = get_fitness(Q)
mp = 1.0;
mc = 2.0;
l = 0.5;
g = 9.81;
Hl = 1/((mc+mp)*mp*(l^2) - mp^2*l^2) ...
    * [mp*l^2, mp*l; mp*l, mc+mp];
Hl_inv = inv(Hl);
Ac = [0, 0, 1, 0; 0, 0, 0, 1;
    0, Hl_inv(1,2)*mp*g*l, 0, 0;
    0, Hl_inv(2,2)*mp*g*l, 0, 0];
Bc = [0; 0; Hl_inv(1,1); Hl_inv(2,1)];

R = 1;

[K,S,P] = lqr(Ac, Bc, Q, R);

%%
function dx = model_fitness(t, x)
% ϵͳ״̬����
mp = 1.0;
mc = 2.0;
l = 0.5;
g = 9.81;
Hl = 1/((mc+mp)*mp*(l^2) - mp^2*l^2) ...
    * [mp*l^2, mp*l; mp*l, mc+mp];
Hl_inv = inv(Hl);
Ac = [0, 0, 1, 0; 0, 0, 0, 1;
    0, Hl_inv(1,2)*mp*g*l, 0, 0;
    0, Hl_inv(2,2)*mp*g*l, 0, 0];
Bc = [0; 0; Hl_inv(1,1); Hl_inv(2,1)];

u = -K*x;
dx = Ac*x + Bc*u;
end

%% ���΢�ַ���
y0 = [0; pi/2; 0; 0];
opt = odeset; opt.RelTol = 1e-8;
[t, x] = ode45(@model_fitness, [0,150], y0, opt);

%% ������Ӧ�Ⱥ��� ԽСԽ��
J = 0;
for i = 1:size(t,1)-1
    delta_t = t(i+1) - t(i);
    xt = x(i+1, :)';
    ut = -K*xt;
    % func_val = xt'*Q*xt + ut*R*ut;
    func_val = xt'*xt + ut*ut;
    % func_val = xt(1)^2 + xt(2)^2 + ut^2;
    J = J + func_val*delta_t;
end
end

%%
function dx = model(t, x)
% ϵͳ״̬����
mp = 1.0;
mc = 2.0;
l = 0.5;
g = 9.81;

Hl = 1/((mc+mp)*mp*(l^2) - mp^2*l^2) ...
    * [mp*l^2, mp*l; mp*l, mc+mp];
Hl_inv = inv(Hl);
Ac = [0, 0, 1, 0; 0, 0, 0, 1;
    0, Hl_inv(1,2)*mp*g*l, 0, 0;
    0, Hl_inv(2,2)*mp*g*l, 0, 0];
Bc = [0; 0; Hl_inv(1,1); Hl_inv(2,1)];

K = [-1.0158  -25.8998   -3.1165  -27.6016];


u = -K*x;
dx = Ac*x + Bc*u;
end