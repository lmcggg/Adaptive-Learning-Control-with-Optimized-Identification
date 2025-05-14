classdef LinearExplorationPolicy
    % LinearExplorationPolicy: Implements a linear exploration policy
    % This is a simple policy that can be parameterized for optimization
    
    properties
        K           % Feedback gain matrix
        mu          % Constant offset
        system      % Reference to the system for dimensions
    end
    
    methods
        function obj = LinearExplorationPolicy(params, system)
            % Constructor for LinearExplorationPolicy
            % params: Policy parameters [K_flat; mu]
            % system: Instance of NonlinearSystem
            
            % Get dimensions
            d_X = system.d_X;
            
            % Extract K and mu from flattened parameters
            K_flat = params(1:d_X*d_X);
            mu = params(d_X*d_X+1:end);
            
            % Reshape K to matrix form
            obj.K = reshape(K_flat, d_X, d_X);
            obj.mu = mu;
            obj.system = system;
        end
        
        function U = control_input(obj, X, ~)
            % Compute control input using linear feedback plus offset
            % X: Current state
            % Returns: Control input U
            
            % Simple linear policy with optional exploration noise
            U = obj.K * X + obj.mu;
        end
    end
    
    methods (Static)
        function params_init = initialize_random(d_X)
            % Initialize random policy parameters
            % d_X: State dimension
            
            % Random K matrix (stabilizing)
            K_init = -0.1 * eye(d_X) + 0.05 * randn(d_X, d_X);
            
            % Random bias
            mu_init = 0.2 * randn(d_X, 1);
            
            % Flatten parameters
            params_init = [K_init(:); mu_init];
        end
    end
end