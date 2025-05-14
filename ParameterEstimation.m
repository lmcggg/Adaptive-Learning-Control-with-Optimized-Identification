function phi_hat = ParameterEstimation(data, phi_init, B, system)
    % ParameterEstimation: Estimate parameters using nonlinear least squares
    % data: Cell array of {X_t, U_t, X_t+1} tuples
    % phi_init: Initial parameter guess
    % B: Bound on parameter norm
    % system: Instance of NonlinearSystem class for dynamics
    
    % Set optimization options
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                          'SpecifyObjectiveGradient', false, ...
                          'MaxIterations', 1000, 'MaxFunctionEvaluations', 5000);
    
    % Reshape phi_init to a vector for optimization
    [d_phi, d_X] = size(phi_init);
    phi_vec_init = reshape(phi_init, [], 1);
    
    % Define objective function (squared prediction error)
    function [loss, grad] = objFun(phi_vec)
        % Reshape parameter vector back to matrix form
        phi = reshape(phi_vec, d_phi, d_X);
        
        % Initialize loss
        loss = 0;
        
        % Accumulate loss over all data points
        for i = 1:length(data)
            X_t = data{i}.X_t;
            U_t = data{i}.U_t;
            X_next_true = data{i}.X_next;
            
            % Predict next state without noise
            X_next_pred = X_t + U_t;
            for j = 1:d_phi
                X_next_pred = X_next_pred + system.psi(X_t - phi(j,:)');
            end
            
            % Compute squared error
            error_vec = X_next_true - X_next_pred;
            loss = loss + error_vec' * error_vec;
        end
        
        % No analytical gradient provided
        grad = [];
    end
    
    % Define constraint function (norm bound)
    function [c, ceq] = constraintFun(phi_vec)
        c = norm(phi_vec) - B;  % ||phi|| <= B
        ceq = [];  % No equality constraints
    end
    
    % Run constrained optimization
    phi_vec_hat = fmincon(@objFun, phi_vec_init, [], [], [], [], ...
                        [], [], @constraintFun, options);
    
    % Reshape result back to matrix form
    phi_hat = reshape(phi_vec_hat, d_phi, d_X);
end 