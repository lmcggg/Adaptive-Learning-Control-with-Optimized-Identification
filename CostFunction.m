function cost = CostFunction(X, U, Q, R, X_ref)
    % CostFunction: Compute the cost for a state-action pair
    % X: State
    % U: Control input
    % Q: State cost weight matrix
    % R: Control cost weight matrix
    % X_ref: Reference state (optional)
    
    % Default reference state is zero if not provided
    if nargin < 5
        X_ref = zeros(size(X));
    end
    
    % Compute state tracking error cost
    state_cost = (X - X_ref)' * Q * (X - X_ref);
    
    % Compute control cost
    if ~isempty(U)
        control_cost = U' * R * U;
    else
        control_cost = 0;
    end
    
    % Total cost
    cost = state_cost + control_cost;
end 