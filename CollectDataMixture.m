function data = CollectDataMixture(pi0, pi_exp, gamma, system, episodes, T)
    % CollectDataMixture: Collect data using a mixture of policies
    % pi0: Initial policy
    % pi_exp: Exploration policy
    % gamma: Mixture weight for pi0
    % system: Instance of NonlinearSystem
    % episodes: Number of episodes to collect
    % T: Number of time steps per episode
    
    % Initialize data cell array
    data = cell(1, episodes * T);
    
    % For each episode
    idx = 1;
    for ep = 1:episodes
        % Initialize random state
        X0 = randn(system.d_X, 1);
        X_current = X0;
        
        % Decide which policy to use for this episode
        use_pi0 = (rand() < gamma);
        
        % Select policy
        if use_pi0
            policy = pi0;
        else
            policy = pi_exp;
        end
        
        % Collect trajectory
        for t = 1:T
            % Get action from policy
            U_t = policy.control_input(X_current, zeros(system.d_X, 1));
            
            % Get next state using system dynamics
            X_next = system.f(X_current, U_t, system.phi_star);
            
            % Store transition
            data{idx}.X_t = X_current;
            data{idx}.U_t = U_t;
            data{idx}.X_next = X_next;
            idx = idx + 1;
            
            % Update current state
            X_current = X_next;
        end
    end
end 