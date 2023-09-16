#  Time-varying Linear Quadratic Regulator (TVLQR)

## Theory

Many systems around us may be estimated to a great extent using time-varying systems. Time-varying Linear Quadratic Regulator(TVLQR) is an expansion of the LQR idea for systems with time-varying dynamics and cost functions. If a non-linear system dynamics may be expressed in the state-space model, it may be linearized and approximated using Taylor series expansion around an operating point, for state $\mathbf{x}_0$ and input $\mathbf{u}_0$. $\mathbf{A}(t)$ and $\mathbf{B}(t)$ are time varying state and input matrices.

```math
\begin{equation*}
    \dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t))
\end{equation*}
```
```math
\begin{align*}
    \dot{\overline{\mathbf{x}}}(t) \approx f(\mathbf{x}_0, \mathbf{u_0}) + \dfrac{\partial f(\mathbf{x}_0, \mathbf{u_0})}{\partial \mathbf{x}}\overline{\mathbf{x}} + \dfrac{\partial f(\mathbf{x}_0, \mathbf{u_0})}{\partial \mathbf{u}}\overline{\mathbf{u}} = \mathbf{A}(t)\overline{\mathbf{x}}(t) + \mathbf{B}(t)\overline{\mathbf{u}}(t) 
    \\
    \text{with $\overline{\mathbf{x}}=\mathbf{x}-\mathbf{x_0}$, $\overline{\mathbf{u}}=\mathbf{u}-\mathbf{u_0}
    $}
\end{align*}
```

TVLQR minimizes a time-varying cost function, $J(t)$, as it optimizes a varying control input, $\mathbf{u}(t)$. $\mathbf{Q}(t)$, $\mathbf{R}(t)$ and $\mathbf{Q}_{f}(t)$ are weighting matrices and play a major role in stabilizing the trajectories. These are the parameters of the TVLQR that need to be tuned to have the trajectory stabilized. The $\mathbf{Q}(t)$ and $\mathbf{R}(t)$ matrices are weighting matrices for the state and control input respectively. Varying the elements in these matrices could affect how strictly the deviation errors in respective components of state and input are minimized. $\mathbf{Q}_{f}(t)$ represents the final state weighing matrix.
```math
\begin{equation*}
    J(t) = \mathbf{x}^T(t_f) \mathbf{Q}_f \mathbf{x}(t_f) + \int_{t_0}^{t_f} \left[ \mathbf{x}^T(t) \mathbf{Q}(t) \mathbf{x}(t) + \mathbf{u}^T(t) \mathbf{R}(t) \mathbf{u}(t) \right] dt
\end{equation*}
```
The optimal time-varying feedback matrix $\mathbf{K}(t)$ is obtained by solving the time-varying Riccati equation, which is obtained by substituting the control law into the linearized dynamics equation.
```math
\begin{equation*}
    \mathbf{u}(t) = -\mathbf{K}(t)\overline{\mathbf{x}}(t)
\end{equation*}
```
```math
\begin{equation*}
    \dot{\mathbf{P}}(t) = -\mathbf{P}(t)\mathbf{A}(t) - \mathbf{A}^T(t)\mathbf{P}(t) + \mathbf{P}(t)\mathbf{B}(t)\mathbf{R}^{-1}(t)\mathbf{B}^T(t)\mathbf{P}(t) - \mathbf{Q}(t)
\end{equation*}
```

Like the LQR, TVLQR relies on the accuracy of the mathematical model. Mismatch between the mathematical model and the robot could impact the performance of the real system. Approximation of highly non-linear systems could also add challenges to the system. However, by considering the dynamics of the robot, the TVLQR allows accurate control with error compensation around the region of control. Being a model-based controller, TVLQR adds to the advantages of the LQR as it extends to time-varying systems.


## Dependencies

The trajectory optimization using direct collocation and TVLQR gains are obtained by taking advantage of [Drake toolbox [2]](https://drake.mit.edu/).


## References

[[1] Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832).](http://underactuated.mit.edu/)

[[2] Model-Based Design and Verification for Robotics](https://drake.mit.edu/).
