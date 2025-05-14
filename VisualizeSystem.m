%% Visualize Nonlinear System Dynamics
% This script visualizes the nonlinear dynamics to better understand the system

% Create system instance
d_X = 2;                  % State dimension
d_phi = 4;                % Number of parameters
sigma_w = 0.1;            % Process noise

% True parameters (example)
phi_star = [1.5, 0.5;     
           -1.0, 1.0;     
            0.5, -1.5;    
           -0.5, -0.5];   

system = NonlinearSystem(phi_star, 0);  % Zero noise for visualization

% Create a grid of points in state space
[X1, X2] = meshgrid(linspace(-3, 3, 20), linspace(-3, 3, 20));
U = zeros(size(X1));  % Zero control input for visualization

% Compute vector field
dx1 = zeros(size(X1));
dx2 = zeros(size(X1));

for i = 1:numel(X1)
    X = [X1(i); X2(i)];
    X_next = system.f(X, [0; 0], phi_star);
    dx1(i) = X_next(1) - X(1);
    dx2(i) = X_next(2) - X(2);
end

% Plot vector field
figure;
quiver(X1, X2, dx1, dx2, 'b');
hold on;

% Plot parameter locations
scatter(phi_star(:,1), phi_star(:,2), 100, 'ro', 'filled');
for i = 1:d_phi
    text(phi_star(i,1), phi_star(i,2), ['\phi_{' num2str(i) '}'], 'FontSize', 12);
end

% Plot nonlinear function effect at a few points
for x1 = -2:2:2
    for x2 = -2:2:2
        X = [x1; x2];
        for i = 1:d_phi
            psi = system.psi(X - phi_star(i,:)');
            quiver(x1, x2, psi(1), psi(2), 'r', 'LineWidth', 1.5);
        end
    end
end

% Generate a few trajectories
t_max = 20;
colors = lines(5);
for traj = 1:5
    X0 = 3 * (rand(2,1) - 0.5);
    X = zeros(2, t_max+1);
    X(:,1) = X0;
    
    for t = 1:t_max
        X(:,t+1) = system.f(X(:,t), [0;0], phi_star);
    end
    
    plot(X(1,:), X(2,:), 'Color', colors(traj,:), 'LineWidth', 1.5);
    scatter(X(1,1), X(2,1), 50, colors(traj,:), 'filled');
    text(X(1,1), X(2,1), ['X_0^{' num2str(traj) '}'], 'FontSize', 10);
end

% Formatting
xlabel('x_1');
ylabel('x_2');
title('Nonlinear System Dynamics');
grid on;
axis equal;
legend('Vector field', 'Parameters', 'Trajectories');

% Create a 3D surface plot of psi function
figure;
[X1, X2] = meshgrid(linspace(-3, 3, 50), linspace(-3, 3, 50));
Z1 = zeros(size(X1));
Z2 = zeros(size(X1));

for i = 1:numel(X1)
    X = [X1(i); X2(i)];
    psi_val = system.psi(X);
    Z1(i) = psi_val(1);
    Z2(i) = psi_val(2);
end

% Plot psi function magnitude
psi_mag = sqrt(Z1.^2 + Z2.^2);
surf(X1, X2, psi_mag);
colormap('jet');
colorbar;
xlabel('x_1');
ylabel('x_2');
zlabel('|\psi(x)|');
title('Magnitude of \psi(x) Function');
grid on; 