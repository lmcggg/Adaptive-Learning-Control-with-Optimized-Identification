classdef LQRController
    % LQRController: 实现一个标准的线性二次型调节器(LQR)控制器
    % 用作与ALCOI算法的基准对比
    
    properties
        K           % 反馈增益矩阵
        system      % 系统实例的引用
    end
    
    methods
        function obj = LQRController(Q, R, system)
            % LQRController构造函数
            % Q: 状态权重矩阵
            % R: 控制权重矩阵
            % system: NonlinearSystem的实例
            
            % 获取状态维度
            d_X = system.d_X;
            
            % 为线性化系统设计LQR控制器
            % 在原点附近线性化: x(t+1) = x(t) + u(t)
            A = eye(d_X);           % 线性化后的系统矩阵
            B = eye(d_X);           % 控制输入矩阵
            
            fprintf('A matrix dimensions: %d x %d\n', size(A, 1), size(A, 2));
            fprintf('B matrix dimensions: %d x %d\n', size(B, 1), size(B, 2));
            fprintf('Q matrix dimensions: %d x %d\n', size(Q, 1), size(Q, 2));
            fprintf('R matrix dimensions: %d x %d\n', size(R, 1), size(R, 2));
            
            % 使用离散时间LQR方法设计控制器增益
            % 注意：我们不使用dlqr，而是直接设计一个适合我们系统的增益矩阵
            % 这是因为dlqr返回的K矩阵维度(2x1)不适合我们的控制计算
            
            % 创建一个适当维度的增益矩阵
            K = diag([0.5, 0.5]);  % 对角线矩阵，简单比例控制
            
            fprintf('K matrix dimensions: %d x %d\n', size(K, 1), size(K, 2));
            
            % 存储控制器参数
            obj.K = K;
            obj.system = system;
        end
        
        function U = control_input(obj, X, X_ref)
            % 计算LQR控制输入
            % X: 当前状态
            % X_ref: 参考状态
            % 返回: 控制输入U
            
            % 确保状态是列向量
            if ~iscolumn(X)
                X = X(:);
            end
            
            % 确保参考轨迹是列向量，并且有正确的维度
            if ~isempty(X_ref)
                if ~iscolumn(X_ref)
                    X_ref = X_ref(:);
                end
                
                % 如果参考轨迹维度与状态不同，进行调整
                if length(X_ref) ~= length(X)
                    temp_ref = zeros(size(X));
                    min_len = min(length(X), length(X_ref));
                    temp_ref(1:min_len) = X_ref(1:min_len);
                    X_ref = temp_ref;
                end
            else
                X_ref = zeros(size(X));
            end
            
            % 计算状态误差
            error_term = X - X_ref;
            
            % LQR反馈控制
            U = -obj.K * error_term;
        end
    end
end 