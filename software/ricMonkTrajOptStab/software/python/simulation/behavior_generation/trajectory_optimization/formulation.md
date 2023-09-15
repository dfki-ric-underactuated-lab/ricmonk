# Trajectory optimization formulation
Taking advantage of the Direct Collocation method by the [Drake](https://drake.mit.edu/) toolbox, and the fixed-base model of the robot, trajectories are optimized for the swing phases using the Sparse Nonlinear OPTimizer([SNOPT](https://www.jstor.org/stable/20453604)). The optimization problem is formulated as follows:

```math
\begin{align*}
    \text{Minimize } \quad & J = W(t_f) + \sum_{n=0}^{N} R(T_1(u_1[n])^2 + T_2(u_2[n])^2) + K(V_1(q_2[n])^2 + V_2(q_3[n])^2) \\
    \text{s.t. } \quad & \text{Dynamics: } \dot{\mathbf{x}}(t_n) = f(\mathbf{x}(t_n), \mathbf{u}(t_n)) \\
    & \text{Costate dynamics: } \dot{\lambda}(t) = -\frac{\partial H}{\partial \mathbf{x}} \\
    & \text{Hamiltonian: } H = L(\mathbf{x}(t_n), \mathbf{u}(t_n)) + \lambda^T f(\mathbf{x}(t_n), \mathbf{u}(t_n)) \\
    & \text{Boundary conditions: } \begin{aligned}[t] 
        \mathbf{x}[0] &= \mathbf{x}_0, \\
        \mathbf{x}[t_f] & \text{ defined using forward geometry and kinematics} \\
        \end{aligned}\\
    & \text{Time step constraint: } 0.005 \leq h[n] \leq 0.5 \\
    & \text{Torque constraints: } |\mathbf{u}[n]| \leq \tau_{\max} \\
    & \text{Velocity constraints: } |\dot{q}_1[n]|, |\dot{q}_2[n]|, |\dot{q}_3[n]| \leq \omega_{\max} \\
    & \text{Position constraints: } |q_1[n]| \leq q_{lim, 1}, |q_2[n]| \leq q_{lim, 2}, |q_3[n]| \leq q_{lim, 3} \\
    & \text{Collision constraints defined using forward geometry}\\
    & \text{Optimization problem warm-started using previously generated trajectories}
\end{align*} 
```
```math
\begin{equation*}
    \mathbf{x} = \begin{bmatrix}
        q_1 \\ q_2 \\ q_3 \\ \dot{q_1} \\ \dot{q_2} \\ \dot{q_3}
    \end{bmatrix} = \begin{bmatrix}
        \mathbf{q} \\ \dot{\mathbf{q}}
    \end{bmatrix}
\end{equation*}
```
```math
\begin{equation*}
    \mathbf{u} = \begin{bmatrix}
        u_1 \\ u_2
    \end{bmatrix}
\end{equation*}   
```

$\mathbf{x}$ and $\mathbf{u}$ are the state vector and the torque vector respectively, and they are the decision variables of the current trajectory optimization problem. $N$ is the number of collocation points that consist the trajectory. $\mathbf{h}$ is a vector that contains the timesteps that separates the collocation points. $\tau_{max}$ is the imposed motor torque limit and $\omega_{max}$ is the imposed motor angular velocity limit for safe operation of the robot. $q_{lim, 1}$, $q_{lim, 2}$, $q_{lim, 3}$ are the position limits for each of the joint angles. In the cost equation(\ref{eq:costBrachTrajOpt}), $R$, $T_1$ and $T_2$ input regularization term and weighting factors each of the actuators respectively. Also in the same equation, $K$, $V_1$ and $V_2$ are velocity regularization term and weighting factors for each of the actuators respectively. $W$ is the time penalization constant. Boundary condition for the final time is defined using the forward geometry and kinematics, such that the free end of $arm_2$ catches is at the required position with the required velocity. From the model we see that the connection of the tail to the arms leads to a kinematic coupling that is not purely serial. Consequently, the inverse kinematics alone cannot directly yield the robot's final state, denoted as $\mathbf{x}(t_f)$, which is a crucial requirement for trajectory optimization.

To determine $\mathbf{x}(t_f)$, a methodology is employed involving the desired final spatial position and velocity of the swing arm's gripper, denoted as $\mathbf{c}^{*}_{f}$. In each iteration, the solver computes the system's forward kinematics based on the decision variable of the state, represented as $\mathbf{c}_{f}$, and strives to minimize the error relative to $\mathbf{c}_{f}^{*}$. Hence, the final position and velocity of the $tail$ is not explicitly constrained. Collision constraints with respect to the free ends of the $tail$ and the $arm_2$ are also defined using forward geometry of the robot.