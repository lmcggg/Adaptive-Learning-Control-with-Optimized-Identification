%% ALCOI Algorithm Simulation
% This script demonstrates the ALCOI (Adaptive Learning Control with
% Optimized Identification) algorithm on a nonlinear dynamical system.
% This implementation aligns with the ALCOI paper, ensuring that:
% 1. The Fisher Information is calculated using estimated parameters, not true ones
% 2. The Model-Task Hessian properly differentiates with respect to system dynamics
%    parameters while keeping the policy fixed based on current estimate
% 3. The nonlinear function psi can be configured to match paper or use robust alternative

%% Parameters
rng(42);  % For reproducibility

% System parameters
d_X = 2;                  % State dimension
d_phi = 4;                % Number of parameters
sigma_w = 0.1;            % Process noise

% True parameters (unknown to the algorithm)
phi_star = [1.5, 0.5;     
            -1.0, 1.0;    
            0.5, -1.5;    
            -0.5, -0.5];  

% Cost function parameters
Q = eye(d_X);            % State cost weight
R = 0.1 * eye(d_X);      % Control cost weight
cost_params = struct('Q', Q, 'R', R);

% ALCOI parameters
N = 50;                   % Total number of episodes
gamma = 0.5;              % Exploration mixture ratio
nu = 0.1;                 % Regularization parameter
epsilon = 1e-4;           % Optimization tolerance
T = 20;                   % Time horizon

% Choose psi function type: 'original' (default) or 'paper'
psi_type = 'paper';  % Use 'paper' to align with the paper's ψ(x) = 5 exp(-x²)

%% Initialize system and initial policy
% Create the nonlinear system with specified psi function type
system = NonlinearSystem(phi_star, sigma_w, psi_type);

% Initialize a simple initial policy (linear feedback)
K0 = -0.5 * eye(d_X);
phi0 = zeros(d_phi, d_X);  % Initial guess for parameters
pi0 = FeedbackLinearizationController(K0, phi0, system);

% 添加标准LQR控制器作为对比
lqr_controller = LQRController(Q, R, system);

%% Run ALCOI algorithm
fprintf('Running ALCOI algorithm...\n');
fprintf('Using psi function type: %s\n', psi_type);
[controller_alcoi, phi_hat_plus] = ALCOI(pi0, system, N, gamma, nu, epsilon, T, cost_params);

%% Evaluate policies
fprintf('Evaluating policies...\n');

% Test scenarios
n_test = 10;          % Number of test episodes
T_test = 50;          % Test time horizon
X0_test = randn(d_X, 1);  % Test initial state

% Reference trajectory (can be modified as needed)
X_ref = zeros(d_X, T_test+1);  % Zero reference 

% Evaluate initial policy
[X_traj_initial, U_traj_initial] = system.simulate(pi0, X0_test, T_test, X_ref);

% Evaluate ALCOI policy
[X_traj_alcoi, U_traj_alcoi] = system.simulate(controller_alcoi, X0_test, T_test, X_ref);

% 评估LQR控制器
[X_traj_lqr, U_traj_lqr] = system.simulate(lqr_controller, X0_test, T_test, X_ref);

%% Compute costs
cost_initial = zeros(T_test, 1);
cost_alcoi = zeros(T_test, 1);
cost_lqr = zeros(T_test, 1);  % 添加LQR成本

for t = 1:T_test
    cost_initial(t) = CostFunction(X_traj_initial(:, t), U_traj_initial(:, t), Q, R, X_ref(:, t));
    cost_alcoi(t) = CostFunction(X_traj_alcoi(:, t), U_traj_alcoi(:, t), Q, R, X_ref(:, t));
    cost_lqr(t) = CostFunction(X_traj_lqr(:, t), U_traj_lqr(:, t), Q, R, X_ref(:, t));  % 计算LQR成本
end

% Cumulative costs
cum_cost_initial = cumsum(cost_initial);
cum_cost_alcoi = cumsum(cost_alcoi);
cum_cost_lqr = cumsum(cost_lqr);  % LQR累积成本

fprintf('Initial policy cumulative cost: %.4f\n', cum_cost_initial(end));
fprintf('ALCOI policy cumulative cost: %.4f\n', cum_cost_alcoi(end));
fprintf('LQR policy cumulative cost: %.4f\n', cum_cost_lqr(end));  % 输出LQR累积成本

%% 计算控制目标误差（状态跟踪误差）

error_initial = zeros(T_test, 1);
error_alcoi = zeros(T_test, 1);
error_lqr = zeros(T_test, 1);

for t = 1:T_test
    error_initial(t) = norm(X_traj_initial(:, t) - X_ref(:, t));
    error_alcoi(t) = norm(X_traj_alcoi(:, t) - X_ref(:, t));
    error_lqr(t) = norm(X_traj_lqr(:, t) - X_ref(:, t));
end

% 累积误差
cum_error_initial = cumsum(error_initial);
cum_error_alcoi = cumsum(error_alcoi);
cum_error_lqr = cumsum(error_lqr);

% 输出最终累积误差
fprintf('\n控制目标误差比较（状态跟踪误差）:\n');
fprintf('Initial policy累积误差: %.4f\n', cum_error_initial(end));
fprintf('ALCOI policy累积误差: %.4f\n', cum_error_alcoi(end));
fprintf('LQR policy累积误差: %.4f\n', cum_error_lqr(end));

%% Plotting results
% Plot trajectories
figure;
subplot(2, 1, 1);
plot(0:T_test, X_traj_initial(1, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(0:T_test, X_traj_alcoi(1, :), 'r--', 'LineWidth', 1.5);
plot(0:T_test, X_traj_lqr(1, :), 'g-.', 'LineWidth', 1.5); 
plot(0:T_test, X_ref(1, :), 'k:', 'LineWidth', 1);
hold off;
xlabel('Time step');
ylabel('State x_1');
legend('Initial policy', 'ALCOI policy', 'LQR policy', 'Reference'); 
title('State Trajectories - x_1');
grid on;

subplot(2, 1, 2);
plot(0:T_test, X_traj_initial(2, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(0:T_test, X_traj_alcoi(2, :), 'r--', 'LineWidth', 1.5);
plot(0:T_test, X_traj_lqr(2, :), 'g-.', 'LineWidth', 1.5);  
plot(0:T_test, X_ref(2, :), 'k:', 'LineWidth', 1);
hold off;
xlabel('Time step');
ylabel('State x_2');
legend('Initial policy', 'ALCOI policy', 'LQR policy', 'Reference');  
title('State Trajectories - x_2');
grid on;

% Plot control inputs
figure;
subplot(2, 1, 1);
plot(1:T_test, U_traj_initial(1, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, U_traj_alcoi(1, :), 'r--', 'LineWidth', 1.5);
plot(1:T_test, U_traj_lqr(1, :), 'g-.', 'LineWidth', 1.5); 
hold off;
xlabel('Time step');
ylabel('Control u_1');
legend('Initial policy', 'ALCOI policy', 'LQR policy'); 
title('Control Inputs - u_1');
grid on;

subplot(2, 1, 2);
plot(1:T_test, U_traj_initial(2, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, U_traj_alcoi(2, :), 'r--', 'LineWidth', 1.5);
plot(1:T_test, U_traj_lqr(2, :), 'g-.', 'LineWidth', 1.5); 
hold off;
xlabel('Time step');
ylabel('Control u_2');
legend('Initial policy', 'ALCOI policy', 'LQR policy');  % 更新图例
title('Control Inputs - u_2');
grid on;

% Plot costs
figure;
subplot(2, 1, 1);
plot(1:T_test, cost_initial, 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, cost_alcoi, 'r--', 'LineWidth', 1.5);
plot(1:T_test, cost_lqr, 'g-.', 'LineWidth', 1.5);  % 添加LQR瞬时成本
hold off;
xlabel('Time step');
ylabel('Instantaneous cost');
legend('Initial policy', 'ALCOI policy', 'LQR policy');  % 更新图例
title('Instantaneous Costs');
grid on;

subplot(2, 1, 2);
plot(1:T_test, cum_cost_initial, 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, cum_cost_alcoi, 'r--', 'LineWidth', 1.5);
plot(1:T_test, cum_cost_lqr, 'g-.', 'LineWidth', 1.5);  % 添加LQR累积成本
hold off;
xlabel('Time step');
ylabel('Cumulative cost');
legend('Initial policy', 'ALCOI policy', 'LQR policy');  
title('Cumulative Costs');
grid on;

% 绘制控制目标误差（状态跟踪误差）
figure;
subplot(2, 1, 1);
plot(1:T_test, error_initial, 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, error_alcoi, 'r--', 'LineWidth', 1.5);
plot(1:T_test, error_lqr, 'g-.', 'LineWidth', 1.5);
hold off;
xlabel('Time step');
ylabel('State tracking error');
legend('Initial policy', 'ALCOI policy', 'LQR policy');
title('Instantaneous State Tracking Error');
grid on;

subplot(2, 1, 2);
plot(1:T_test, cum_error_initial, 'b-', 'LineWidth', 1.5);
hold on;
plot(1:T_test, cum_error_alcoi, 'r--', 'LineWidth', 1.5);
plot(1:T_test, cum_error_lqr, 'g-.', 'LineWidth', 1.5);
hold off;
xlabel('Time step');
ylabel('Cumulative error');
legend('Initial policy', 'ALCOI policy', 'LQR policy');
title('Cumulative State Tracking Error');
grid on;

%% Parameter estimation error

param_error = norm(reshape(phi_hat_plus - phi_star, [], 1));
fprintf('Parameter estimation error: %.4f\n', param_error);


figure;
subplot(2, 2, 1);
scatter(phi_star(:, 1), phi_star(:, 2), 100, 'bo', 'filled');
hold on;
scatter(phi_hat_plus(:, 1), phi_hat_plus(:, 2), 100, 'ro');
for i = 1:d_phi
    plot([phi_star(i, 1), phi_hat_plus(i, 1)], [phi_star(i, 2), phi_hat_plus(i, 2)], 'k--');
    text(phi_star(i, 1), phi_star(i, 2), ['\phi_{' num2str(i) '}^*'], 'FontSize', 12);
    text(phi_hat_plus(i, 1), phi_hat_plus(i, 2), ['\phi_{' num2str(i) '}'], 'FontSize', 12);
end
hold off;
xlabel('\phi_1');
ylabel('\phi_2');
legend('True parameters', 'Estimated parameters');
title(['Parameter Estimation Comparison (psi type: ' psi_type ')']);
grid on;
axis equal;

%% Save results
save('alcoi_results.mat', 'phi_star', 'phi_hat_plus', 'X_traj_initial', 'X_traj_alcoi', ...
     'U_traj_initial', 'U_traj_alcoi', 'cost_initial', 'cost_alcoi', 'error_initial', 'error_alcoi', ...
     'error_lqr', 'cum_error_initial', 'cum_error_alcoi', 'cum_error_lqr', 'psi_type'); 