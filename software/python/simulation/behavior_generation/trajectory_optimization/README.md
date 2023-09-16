<div align="center">

# RicMonk Trajectory Optmization
</div>

## Fixed-base model
To optimize trajectories for brachiation, in order to simplify the system and to improve the probability of finding optimal solutions, we use a fixed-base model of the robot as shown in the following figure.
<div align="center">
<img width="500" src="../../../../../../../data/model/ricMonkFullSchematic.png" />
</div>
In such a model for the RicMonk, the joint between $arm_1$ and ladder bar at $\mathbf{(y_{c}, z_{c})}$ is a revolute joint(dotted circle in figure). In reality, the arm in contact with the support bar can move if enough force is applied on the arm. Making use of such a model reduces the number of constraints to be imposed, hence simplifying the problem. However, use of such a fixed-base model has its own disadvantages. $arm_1$ cannot let go of the bar, hence trajectories where that happens may not be optimized using this model.

The study on [AcroMonk](https://arxiv.org/abs/2305.08373), a two-linked brachiation robot, that is capable of continuously performing robust multiple brachiation maneuvers describes the use of a state machine. The state machine is an implementation that helps the robot to continually brachiate either forwards or backwards while taking failure of brachiation into account. For instance, when the robot is not able to successfully catch the bar, when the robot looses all contact with the bar and falls on the ground, when the arm slips off the ladder bar, etc. Given a situation with the robot in any of these cases, the robot either tries to brachiate again using some recovery trajectories or shuts down the actuators to keep the user, the environment and the robot itself safe. To have such an efficient state machine, the complex brachiation motion was broken down to four distinct simpler maneuvers(a.k.a. atomic behaviours), **ZB**, **ZF**, **BF** and **FB**. These maneuvers translate respectively to 'zero to back', 'zero to front', 'back to front' and 'front to back'. The forward direction of motion is  considered to be the direction in which the hook forms a concave shape. Each of these trajectories are illustrated in the following section. Such an implementation of a state machine enables the robot to perform continuous robust brachiation, even in the face of disturbances. In addition, it simplifies the task of optimizing trajectories. 

Formulation of the trajectory optimization problem is described in [here](formulation.md)


## Atomic Behaviors
The swing trajectories for these atomic behaviours are optimized and the animations are obtained as follows. To release the grasp of respective arms for forward or backward brachiation, feed-forward torque is applied.

<!-- <div align="center">
<img width="500" src="../../../../../../../data/model/stateMachine.png" />
</div> -->

<div align="center">
<img width="200" src="../../../../../../../hardware/imagesAndGifs/ZB_ini_gif.gif" >
<img width="200" src="../../../../../../../hardware/imagesAndGifs/ZF_ini_gif.gif" >
<img width="200" src="../../../../../../../hardware/imagesAndGifs/BF_ini_gif.gif" >
<img width="200" src="../../../../../../../hardware/imagesAndGifs/FB_ini_gif.gif" >
</div>

## Operation
The folder **direct_collocation** contains files that provide the respective hyper-parameters for optimizing trajectories. The file **simulateTrajOpt.py** is to run for optimizing trajectories. Input the name of the trajectory (ZB, ZF, BF, FB) and the ladder bar distance (default = 0.34m). The optimized trajectories are saved in the folder with the name of the maneuver in [here](/data/trajectories/direct_collocation/) **Check!!!**  **../../../../../data/trajectories/direct_collocation/** with respect to the current folder. 