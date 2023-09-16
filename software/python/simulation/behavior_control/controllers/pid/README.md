#  Proportional–Integral–Derivative (PID) Control 

## Theory 

The Proportional-Integral-Derivative(PID) controller is a model-free controller, that generates the amount of torque($\mathbf{u}$) supplied by the motors to compensate for the error in the generalized positions($\mathbf{q}$) and velocities($\dot{\mathbf{q}}$) of the robot. The gains $K_p$, $K_i$, and $K_d$ are the proportional, integral, and derivative gains that control the impact of error compensation in generalized positions and velocities respectively. If the desired position and velocity vectors are represented by $\mathbf{q}^*$ and $\dot{\mathbf{q}}^*$, the following equation presents the governing equation of the PID controller.

```math
\begin{equation*}
    \mathbf{u}(t) = K_p \cdot (\mathbf{q}^*(t) - \mathbf{q}(t)) + K_i \cdot \int_{0}^{t} (\mathbf{q}^*(t) - \mathbf{q}(t)) dt + K_d \cdot (\dot{\mathbf{q}}^*(t) - \dot{\mathbf{q}}(t))
\end{equation*}
```

PID controllers are simple to implement. They are efficient in reducing steady errors and they improve system stability. However, the three gains that are required to be manually tuned may pose an error while tuning.  On the other hand, they do not require the model of the system, and their performance is not related to the accuracy of the mathematical model. They may be applied to various systems, and they deliver robust performances.


## Dependencies

The trajectory optimization using direct collocation and PID gains are obtained by taking advantage of [Drake toolbox [2]](https://drake.mit.edu/).


## References

[[1] Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832).](http://underactuated.mit.edu/)

[[2] Model-Based Design and Verification for Robotics](https://drake.mit.edu/).