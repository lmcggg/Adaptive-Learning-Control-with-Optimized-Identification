classdef FeedbackLinearizationController
    % FeedbackLinearizationController: Implements a feedback linearization controller
    
    properties
        K           % Feedback gain matrix
        phi         % Parameter estimates
        system      % Instance of NonlinearSystem for psi function
    end
    
    methods
        function obj = FeedbackLinearizationController(K, phi, system)
            % Constructor for FeedbackLinearizationController
            obj.K = K;
            obj.phi = phi;
            obj.system = system;
        end
        
        function U = control_input(obj, X, X_ref)
            % Compute control input using feedback linearization
            % X: Current state
            % X_ref: Reference state (possibly of different dimensions)
            % Returns: Control input U
            
            % 确保X和X_ref维度一致
            if ~isequal(size(X), size(X_ref))
                % 创建与X尺寸相同的临时参考
                temp_ref = zeros(size(X));
                
                % 复制X_ref的值到temp_ref，确保不超出界限
                for i = 1:min(length(X), length(X_ref))
                    temp_ref(i) = X_ref(i);
                end
                
                X_ref = temp_ref;
            end
            
            % Compute linear feedback term
            U = obj.K * (X - X_ref);
            
            % Subtract nonlinear terms using estimated parameters
            for i = 1:size(obj.phi, 1)
                U = U - obj.system.psi(X - obj.phi(i,:)');
            end
        end
    end
end 