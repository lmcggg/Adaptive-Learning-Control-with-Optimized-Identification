# ALCOI Algorithm Implementation in MATLAB

This repository contains a MATLAB implementation of the Adaptive Learning Control with Optimized Identification (ALCOI) algorithm. The algorithm optimizes the exploration policy for system identification to minimize control costs in nonlinear systems with unknown dynamics.
the paper is “Active Learning for Control-Oriented Identificationof Nonlinear Systems”-Bruce D. Lee, Ingvar Ziemann, George J. Pappas, Nikolai Matni（2024 IEEE 63rd Conference on Decision and Control (CDC)）,thanks for their brilliant idea。

## Algorithm Overview

ALCOI integrates active learning with control optimization to efficiently identify the parameters of an unknown nonlinear dynamical system while optimizing a control objective. The algorithm consists of three key steps:

1. **Initial Exploration**: Collect data using an initial policy to get a preliminary estimate of the system parameters.
2. **Exploration Policy Optimization**: Compute the Model Task Hessian (H) and Fisher Information Matrix (FI), then optimize the exploration policy to minimize the trace of H·FI^(-1). This optimization balances control performance with parameter identification quality.
3. **Mixed Exploration**: Collect additional data using a mixture of the initial policy and the optimized exploration policy, then refine the parameter estimate and synthesize the final controller.

## Technical Details

### System Dynamics
The implementation uses a nonlinear dynamical system defined as:
```
X_{t+1} = X_t + U_t + ∑ ψ(X_t - φ_i) + W_t
```
where:
- X_t is the system state at time t
- U_t is the control input
- φ_i are the unknown parameters to be identified
- ψ(x) is a nonlinear function with two implementations:
  - 'complex': ψ(x) = 5·x/(|x|+ε)·exp(-|x|²)
  - 'paper': ψ(x) = 5·exp(-x²)
- W_t is process noise with standard deviation σ_w

### Parameter Estimation
The algorithm uses constrained nonlinear least squares optimization to estimate the system parameters from collected data. The estimation includes:
- Bounded parameter norms to ensure stability
- Sequential updating of parameter estimates as more data is collected

### Exploration Policy Optimization
The ALCOI algorithm optimizes the exploration policy by:
1. Computing the Model Task Hessian (H) which captures how parameter errors affect control performance
2. Computing the Fisher Information Matrix (FI) which quantifies the informativeness of data
3. Minimizing trace(H·FI^(-1)) to balance control performance and system identification quality

### Controllers
The project implements and compares three controllers:
1. **Initial Controller**: A simple feedback linearization controller with initial parameter estimates
2. **ALCOI Controller**: A feedback linearization controller with optimized parameter estimates
3. **LQR Controller**: A standard Linear Quadratic Regulator for comparison

## Files Description

- `NonlinearSystem.m`: Class implementing the nonlinear dynamical system with unknown parameters.
- `ParameterEstimation.m`: Function for parameter estimation using nonlinear least squares.
- `FeedbackLinearizationController.m`: Class implementing a feedback linearization controller.
- `LQRController.m`: Class implementing a standard Linear Quadratic Regulator controller for comparison.
- `ComputeFisherInformation.m`: Function to compute the Fisher Information Matrix.
- `ComputeModelTaskHessian.m`: Function to compute the Model Task Hessian.
- `OptimizeExplorationPolicy.m`: Function to optimize the exploration policy.
- `LinearExplorationPolicy.m`: Class implementing a parameterized linear exploration policy.
- `CollectData.m`: Function for data collection using specific policies.
- `CollectDataMixture.m`: Function for data collection using a mixture of policies.
- `CostFunction.m`: Function to compute the control cost.
- `ALCOI.m`: Main function implementing the ALCOI algorithm.
- `RunALCOI.m`: Script to run a complete simulation of the ALCOI algorithm.
- `CompareControllers.m`: Script to compare ALCOI with LQR under different noise levels.
- `TrackingTest.m`: Script to compare tracking performance of different controllers.
- `VisualizeSystem.m`: Script to visualize the nonlinear dynamics.

## Usage

1. **Run the basic ALCOI simulation**:
   ```matlab
   RunALCOI
   ```
   This script will run the ALCOI algorithm and compare it with an initial policy and standard LQR control.

2. **Compare controllers under different conditions**:
   ```matlab
   CompareControllers
   ```
   This script evaluates ALCOI and LQR controllers across different noise levels.

3. **Test tracking performance**:
   ```matlab
   TrackingTest
   ```
   This script tests and compares how well different controllers track a circular reference trajectory.

4. **Visualize the system dynamics**:
   ```matlab
   VisualizeSystem
   ```
   This script creates visualizations of the nonlinear system dynamics.

## Key Parameters

Important parameters that can be adjusted in the simulation scripts:
- `N`: Total number of episodes for data collection
- `gamma`: Exploration mixture ratio (balance between initial and optimized exploration)
- `nu`: Regularization parameter for matrix inversions
- `T`: Time horizon for episode simulations
- `sigma_w`: Process noise standard deviation
- `Q` and `R`: Cost function weight matrices for state and control

## Results

The implementation demonstrates several important findings:
1. ALCOI outperforms standard LQR control for nonlinear systems with unknown dynamics
2. The performance advantage of ALCOI increases with higher noise levels
3. ALCOI provides better tracking performance for reference trajectories
4. The optimized exploration policy significantly improves parameter estimation accuracy
![image](https://github.com/user-attachments/assets/350388f2-63a0-4b8e-88c9-d6f13dc3712a)

![image](https://github.com/user-attachments/assets/80c67da7-4684-4095-b1df-fa56cc4b36b3)

![image](https://github.com/user-attachments/assets/1de8953f-78f5-4c57-9039-c5a1a6cbb186)

![image](https://github.com/user-attachments/assets/eb097e70-55e8-42cb-9b0c-01d4c2158b17)

![image](https://github.com/user-attachments/assets/3b7f93c9-8c7d-490e-ab69-7d261fd2fa7a)

Synthesizing final controller
Evaluating policies...
Initial policy cumulative cost: 1485.3375
ALCOI policy cumulative cost: 112.5263
LQR policy cumulative cost: 284.4591

控制目标误差比较（状态跟踪误差）:
Initial policy累积误差: 179.3296
ALCOI policy累积误差: 37.4765
LQR policy累积误差: 111.1595
Parameter estimation error: 10.4670

## Requirements

- MATLAB R2019b or newer
- Optimization Toolbox (for fmincon)
- Control System Toolbox (for dlqr)

## References

The ALCOI algorithm is based on the theory of optimizing the exploration policy for system identification to minimize the control costs in nonlinear systems with unknown dynamics. The approach combines elements from:
- Optimal experimental design
- Active learning
- Nonlinear control theory
- Adaptive control

## my comment

This is a very good method for solving how to generate valid trajectory data in system identification to effectively identify key parameters. Moreover, it performs well and has good reproducibility. However, it requires frequent solving of non-convex optimization problems, which may not be suitable for high-dimensional parameters and complex dynamic models. It needs to satisfy the Lojasiewicz condition, smoothness assumptions, etc., which may be difficult to verify in actual systems.

