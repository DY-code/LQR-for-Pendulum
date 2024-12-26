% https://fuchaojie.com/2022/08/LQR-for-Cartpole/
%%
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

Q = diag([10, 10, 0, 0]);
R = 1;

[K,S,P] = lqr(Ac, Bc, Q, R);

%%
y0 = [0; pi/2; 0; 0];
opt = odeset; opt.RelTol = 1e-8;
% 求解微分方程
figure;
[t, x] = ode45(@model, [0,150], y0, opt);
t_plot = t(1:473);
x_plot = x(1:473, :);
% plot(t_plot, x_plot, '-');
plot(x, '-');
legend('$x$', '$\theta$', '$\dot{x}$', '$\dot{\theta}$', 'Interpreter', 'latex');
xlabel('step');
ylabel('value');

%%
function dx = model(t, x)
% 系统状态方程
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

F = [-3.1623, -58.2683, -8.4835, -63.4380];

u = -F*x;
dx = Ac*x + Bc*u;
end