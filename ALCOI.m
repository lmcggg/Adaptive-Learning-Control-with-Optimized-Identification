function [controller, phi_hat_plus] = ALCOI(pi0, system, N, gamma, nu, epsilon, T, cost_params)
    % ALCOI: Adaptive Learning Control with Optimized Identification
    % pi0: Initial policy
    % system: Instance of NonlinearSystem
    % N: Total number of episodes
    % gamma: Exploration mixture ratio
    % nu: Regularization parameter
    % epsilon: Optimization tolerance
    % T: Time horizon for each episode
    % cost_params: Structure with cost function parameters
    
    % Step 1: Initial policy exploration
    fprintf('Step 1: Initial policy exploration\n');
    n_initial = floor(gamma * N);
    data_initial = CollectData(pi0, system, n_initial, T);
    
    % Initialize parameter estimate near zero
    phi_init = 0.1 * randn(system.d_phi, system.d_X);
    
    % Parameter estimation with constraint
    B = 10;  % Bound on parameter norm
    phi_hat_minus = ParameterEstimation(data_initial, phi_init, B, system);
    fprintf('Initial parameter estimate:\n');
    disp(phi_hat_minus);
    
    % Step 2: Compute H and FI, optimize exploration policy
    fprintf('Step 2: Computing Model Task Hessian and Fisher Information\n');
    
    % Create cost function handle with given parameters
    Q = cost_params.Q;
    R = cost_params.R;
    cost_function = @(X, U) CostFunction(X, U, Q, R);
    
    % Compute model task Hessian
    H = ComputeModelTaskHessian(phi_hat_minus, system, cost_function, T, 10);
    
    % Reshape H to match dimensions of Fisher Information
    [d_phi, d_X] = size(phi_hat_minus);
    d_phi_vec = d_phi * d_X;
    H = reshape(H, d_phi_vec, d_phi_vec);
    
    % Compute Fisher Information Matrix for initial policy using estimated parameters
    FI = ComputeFisherInformation(phi_hat_minus, pi0, system, T, 10);
    
    % Ensure matrices are positive definite
    H = H + 1e-6 * eye(size(H));
    FI = FI + 1e-6 * eye(size(FI));
    
    fprintf('Optimizing exploration policy\n');
    % Create a policy class function handle
    policy_class = @(params, sys) LinearExplorationPolicy(params, sys);
    
    % Initialize policy parameters
    policy_init = LinearExplorationPolicy.initialize_random(system.d_X);
    
    % Set optimization options
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                          'OptimalityTolerance', epsilon);
    
    % Optimize exploration policy - pass phi_hat_minus as the parameter estimate
    pi_exp = OptimizeExplorationPolicy(H, FI, nu, policy_class, system, policy_init, options, phi_hat_minus);
    
    % Step 3: Mixed data collection and final estimation
    fprintf('Step 3: Mixed data collection and final estimation\n');
    n_mixed = floor((1 - gamma) * N);
    data_mixed = CollectDataMixture(pi0, pi_exp, gamma, system, n_mixed, T);
    
    % Combine all data
    data_all = [data_initial, data_mixed];
    
    % Final parameter estimation using all data
    phi_hat_plus = ParameterEstimation(data_all, phi_hat_minus, B, system);
    fprintf('Final parameter estimate:\n');
    disp(phi_hat_plus);
    
    % Synthesize final controller
    fprintf('Synthesizing final controller\n');
    K = -1.0 * eye(system.d_X);  % Simple stabilizing feedback
    controller = FeedbackLinearizationController(K, phi_hat_plus, system);
end 