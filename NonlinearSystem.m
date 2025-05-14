classdef NonlinearSystem
    % NonlinearSystem: Implements a nonlinear dynamical system with unknown parameters
    
    properties
        phi_star    % True parameter values [phi1, phi2, phi3, phi4]
        sigma_w     % Process noise standard deviation
        d_X         % State dimension
        d_phi       % Parameter dimension
        psi_type    % Type of psi function: 'original' (default) or 'paper'
    end
    
    methods
        function obj = NonlinearSystem(phi_star, sigma_w, psi_type)
            % Constructor for NonlinearSystem
            obj.phi_star = phi_star;
            obj.sigma_w = sigma_w;
            [obj.d_phi, obj.d_X] = size(phi_star);
            
            % Default psi function type if not specified
            if nargin < 3
                obj.psi_type = 'original';
            else
                obj.psi_type = psi_type;
            end
        end
        
        function X_next = f(obj, X, U, phi)
            % Nonlinear dynamics function: X_{t+1} = f(X_t, U_t; phi) + W_t
            % X: Current state
            % U: Control input
            % phi: Parameter values [phi1, phi2, phi3, phi4]
            % Returns: Next state including noise
            
            % Pre-allocate the result
            X_next = X + U;
            
            % Apply nonlinear transformation based on phi
            for i = 1:size(phi, 1)
                X_next = X_next + obj.psi(X - phi(i,:)');
            end
            
            % Add process noise
            X_next = X_next + obj.sigma_w * randn(size(X));
        end
        
        function result = psi(obj, x)
            % Nonlinear function implementation
            % Use the selected psi function type
            switch obj.psi_type
                case 'original'
                    % Original implementation: psi(x) = 5*x/(|x|+ε)*exp(-|x|²)
                    norm_x = norm(x);
                    epsilon = 1e-6;
                    result = 5 * x / (norm_x + epsilon) * exp(-norm_x^2);
                
                case 'paper'
                    % Paper implementation (Sec. V): "ψ(x) = 5 exp(-x²)"
                    % Interpreting this as element-wise for vector inputs
                    % to ensure that ψ: R² → R²
                    result = 5 * exp(-x.^2);
                    
                otherwise
                    error('Unknown psi function type');
            end
        end
        
        function [X_trajectory, U_trajectory] = simulate(obj, controller, X0, T, X_ref)
            % Simulate the system for T time steps
            % controller: Control policy object
            % X0: Initial state
            % T: Number of time steps
            % X_ref: Reference trajectory (optional)
            
            % Initialize trajectories
            X_trajectory = zeros(obj.d_X, T+1);
            U_trajectory = zeros(size(X0, 1), T);
            X_trajectory(:, 1) = X0;
            
            % If X_ref is not provided, use zeros
            if nargin < 5
                X_ref = zeros(size(X0, 1), T+1);
            end
            
            % 确保X_ref有正确的维度
            [rows_ref, cols_ref] = size(X_ref);
            if rows_ref ~= size(X0, 1) || cols_ref ~= T+1
                warning('参考轨迹维度不匹配，将被调整以适应模拟要求。');
                
                % 创建正确维度的参考轨迹
                correct_X_ref = zeros(size(X0, 1), T+1);
                
                % 复制现有参考轨迹的值，确保不超出界限
                for i = 1:min(rows_ref, size(X0, 1))
                    for j = 1:min(cols_ref, T+1)
                        correct_X_ref(i, j) = X_ref(i, j);
                    end
                end
                
                X_ref = correct_X_ref;
            end
            
            % Simulate forward
            for t = 1:T
                % Compute control input
                U_trajectory(:, t) = controller.control_input(X_trajectory(:, t), X_ref(:, t));
                
                % Compute next state using true dynamics
                X_trajectory(:, t+1) = obj.f(X_trajectory(:, t), U_trajectory(:, t), obj.phi_star);
            end
        end
    end
end 