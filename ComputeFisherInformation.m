function FI = ComputeFisherInformation(phi, policy, system, T, num_samples)
    % ComputeFisherInformation: Compute the Fisher Information Matrix
    % phi: Current parameter estimate
    % policy: Policy to evaluate (controller object)
    % system: Instance of NonlinearSystem
    % T: Time horizon
    % num_samples: Number of Monte Carlo samples
    
    % Get dimensions
    [d_phi, d_X] = size(phi);
    d_phi_vec = d_phi * d_X;
    
    % Initialize Fisher Information Matrix
    FI = zeros(d_phi_vec, d_phi_vec);
    
    % Monte Carlo simulation to approximate expectation
    for n = 1:num_samples
        % Initialize state randomly
        X0 = randn(d_X, 1);
        
        % Simulate trajectory using the given policy
        Xs = zeros(d_X, T+1);
        Us = zeros(d_X, T);
        Xs(:, 1) = X0;
        
        % Generate trajectory
        for t = 1:T
            % Get control input from policy
            Us(:, t) = policy.control_input(Xs(:, t), zeros(d_X, 1));  % Assuming zero reference
            
            % Get next state without noise
            Xs(:, t+1) = system.f(Xs(:, t), Us(:, t), phi);
        end
        
        % Compute Jacobians for each state-action pair
        jacobian_sum = zeros(d_phi_vec, d_phi_vec);
        
        for t = 1:T
            % Compute Jacobian of f with respect to phi at (X_t, U_t, phi)
            J = ComputeJacobian(Xs(:, t), Us(:, t), phi, system);
            
            % Accumulate outer product
            jacobian_sum = jacobian_sum + J' * J;
        end
        
        % Add to Fisher Information
        FI = FI + (1/system.sigma_w^2) * jacobian_sum;
    end
    
    % Average over samples
    FI = FI / num_samples;
end

function J = ComputeJacobian(X, U, phi, system)
    % ComputeJacobian: Compute the Jacobian of f w.r.t. phi
    % Uses finite differences
    
    % Get dimensions
    [d_phi, d_X] = size(phi);
    
    % Reshape phi to a vector
    phi_vec = reshape(phi, [], 1);
    d_phi_vec = length(phi_vec);
    
    % Initialize Jacobian matrix
    J = zeros(d_X, d_phi_vec);
    
    % Small perturbation
    h = 1e-6;
    
    % Compute base dynamics
    f_base = X + U;
    for i = 1:d_phi
        f_base = f_base + system.psi(X - phi(i,:)');
    end
    
    % Compute Jacobian using finite differences
    for i = 1:d_phi_vec
        % Create perturbed phi
        phi_perturbed = phi_vec;
        phi_perturbed(i) = phi_perturbed(i) + h;
        phi_mat_perturbed = reshape(phi_perturbed, d_phi, d_X);
        
        % Compute perturbed dynamics
        f_perturbed = X + U;
        for j = 1:d_phi
            f_perturbed = f_perturbed + system.psi(X - phi_mat_perturbed(j,:)');
        end
        
        % Finite difference approximation
        J(:, i) = (f_perturbed - f_base) / h;
    end
end 