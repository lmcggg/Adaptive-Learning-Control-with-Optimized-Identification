function H = ComputeModelTaskHessian(phi, system, cost_function, T, num_samples)
    % ComputeModelTaskHessian: Compute the Hessian of the task cost with respect to phi
    % phi: Current parameter estimate (phi_hat_minus)
    % system: Instance of NonlinearSystem
    % cost_function: Function handle taking state and control as input
    % T: Time horizon
    % num_samples: Number of Monte Carlo samples
    
    % Get dimensions
    [d_phi, d_X] = size(phi);
    d_phi_vec = d_phi * d_X;
    
    % Initialize Hessian
    H = zeros(d_phi_vec, d_phi_vec);
    
    % Small perturbation for finite differences
    h = 1e-4;
    
    % Reshape phi to vector for perturbation
    phi_vec = reshape(phi, [], 1);
    
    % Base cost with original parameters
    J_base = EvaluateCost(phi, phi, system, cost_function, T, num_samples);
    
    % First-order perturbations for each parameter
    J_plus = zeros(d_phi_vec, 1);
    
    for i = 1:d_phi_vec
        % Perturb ith parameter positively
        phi_perturbed = phi_vec;
        phi_perturbed(i) = phi_perturbed(i) + h;
        phi_mat_perturbed = reshape(phi_perturbed, d_phi, d_X);
        
        % Evaluate cost with perturbed parameter ONLY for dynamics, not for policy
        J_plus(i) = EvaluateCost(phi, phi_mat_perturbed, system, cost_function, T, num_samples);
    end
    
    % Compute second-order finite differences (Hessian)
    for i = 1:d_phi_vec
        for j = i:d_phi_vec  % Exploit symmetry
            % Create doubly perturbed phi
            phi_perturbed = phi_vec;
            phi_perturbed(i) = phi_perturbed(i) + h;
            phi_perturbed(j) = phi_perturbed(j) + h;
            phi_mat_perturbed = reshape(phi_perturbed, d_phi, d_X);
            
            % Evaluate cost with doubly perturbed parameters ONLY for dynamics, not for policy
            J_plus_plus = EvaluateCost(phi, phi_mat_perturbed, system, cost_function, T, num_samples);
            
            % Compute mixed partial derivative
            if i == j
                % Diagonal element (second derivative)
                phi_perturbed = phi_vec;
                phi_perturbed(i) = phi_perturbed(i) + 2*h;
                phi_mat_perturbed = reshape(phi_perturbed, d_phi, d_X);
                J_plus2 = EvaluateCost(phi, phi_mat_perturbed, system, cost_function, T, num_samples);
                
                % Central difference formula for second derivative
                H(i, i) = (J_plus2 - 2*J_plus(i) + J_base) / (h^2);
            else
                % Off-diagonal element (mixed partial)
                % Use the formula: ∂²f/∂x∂y ≈ [f(x+h,y+h) - f(x+h,y) - f(x,y+h) + f(x,y)]/h²
                phi_perturbed_i = phi_vec;
                phi_perturbed_i(i) = phi_perturbed_i(i) + h;
                phi_mat_perturbed_i = reshape(phi_perturbed_i, d_phi, d_X);
                J_plus_i = EvaluateCost(phi, phi_mat_perturbed_i, system, cost_function, T, num_samples);
                
                phi_perturbed_j = phi_vec;
                phi_perturbed_j(j) = phi_perturbed_j(j) + h;
                phi_mat_perturbed_j = reshape(phi_perturbed_j, d_phi, d_X);
                J_plus_j = EvaluateCost(phi, phi_mat_perturbed_j, system, cost_function, T, num_samples);
                
                H(i, j) = (J_plus_plus - J_plus_i - J_plus_j + J_base) / (h^2);
                H(j, i) = H(i, j);  % Symmetry
            end
        end
    end
end

function J = EvaluateCost(phi_policy, phi_dynamics, system, cost_function, T, num_samples)
    % EvaluateCost: Evaluate the average cost over multiple trajectories
    % phi_policy: Parameter estimate used for controller design (fixed)
    % phi_dynamics: Parameter estimate used for dynamics simulation (perturbed for Hessian)
    % system: System instance
    % cost_function: Cost function handle
    % T: Time horizon
    % num_samples: Number of Monte Carlo samples
    
    % Get dimensions
    [d_phi, d_X] = size(phi_policy);
    
    % Define a feedback controller based on FIXED estimated parameters
    K = -eye(d_X);  % Simple stabilizing feedback
    controller = FeedbackLinearizationController(K, phi_policy, system);
    
    % Initialize total cost
    total_cost = 0;
    
    % Run multiple trajectories
    for n = 1:num_samples
        % Initialize random state
        X0 = randn(d_X, 1);
        
        % Simulate trajectory
        Xs = zeros(d_X, T+1);
        Us = zeros(d_X, T);
        Xs(:, 1) = X0;
        
        % Generate trajectory with perturbed parameters for dynamics (no noise)
        for t = 1:T
            % Get control input using FIXED policy parameters
            Us(:, t) = controller.control_input(Xs(:, t), zeros(d_X, 1));
            
            % Get next state using PERTURBED parameters phi_dynamics
            X_next = Xs(:, t) + Us(:, t);
            for i = 1:d_phi
                X_next = X_next + system.psi(Xs(:, t) - phi_dynamics(i,:)');
            end
            Xs(:, t+1) = X_next;
        end
        
        % Calculate cumulative cost for this trajectory
        traj_cost = 0;
        for t = 1:T
            traj_cost = traj_cost + cost_function(Xs(:, t), Us(:, t));
        end
        traj_cost = traj_cost + cost_function(Xs(:, T+1), zeros(size(Us,1), 1));  % Terminal cost
        
        % Add to total
        total_cost = total_cost + traj_cost;
    end
    
    % Return average cost
    J = total_cost / num_samples;
end 