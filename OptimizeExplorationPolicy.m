function pi_exp = OptimizeExplorationPolicy(H, FI, nu, policy_class, system, policy_init, options, phi_estimate)
    % OptimizeExplorationPolicy: Optimize the exploration policy to minimize
    % tr((H + nu*I) * FI^(-1))
    % H: Model Task Hessian
    % FI: Fisher Information Matrix for initial policy
    % nu: Regularization parameter
    % policy_class: Function handle to create policy with parameters
    % system: Instance of NonlinearSystem
    % policy_init: Initial policy parameters
    % options: Optimization options
    % phi_estimate: Current parameter estimate (phi_hat_minus)
    
    % Get dimensions
    d_param = length(policy_init);
    
    % Default optimization options if not provided
    if nargin < 7
        options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                              'MaxIterations', 500, 'MaxFunctionEvaluations', 1000);
    end
    
    % Regularized Hessian
    H_reg = H + nu * eye(size(H));
    
    % Objective function to minimize: tr((H + nu*I) * FI^(-1))
    function obj = objective(policy_params)
        % Create policy from parameters
        policy = policy_class(policy_params, system);
        
        % Compute Fisher Information for this policy using estimated parameters (not true parameters)
        FI_policy = ComputeFisherInformation(phi_estimate, policy, system, 20, 10);
        
        % Ensure FI is positive definite (add small regularization if needed)
        FI_policy = FI_policy + 1e-6 * eye(size(FI_policy));
        
        % Compute objective: trace(H_reg * FI_policy^(-1))
        % Use matrix properties: trace(AB) = trace(BA)
        obj = trace(H_reg / FI_policy);
    end
    
    % Bounds for policy parameters (problem-specific)
    lb = -5 * ones(d_param, 1);
    ub = 5 * ones(d_param, 1);
    
    % Optimize
    policy_params = fmincon(@objective, policy_init, [], [], [], [], lb, ub, [], options);
    
    % Create optimized policy
    pi_exp = policy_class(policy_params, system);
end 