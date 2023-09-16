<div align="center">

# RicMonk Trajectory Optimization
</div>

## Fixed-base model
To optimize trajectories for brachiation, simplify the system, and improve the probability of finding optimal solutions, we use a fixed-base model of the robot as shown in the following figure.
<div align="center">
<img width="500" src="data/model/ricMonkFullSchematic.png" />
</div>
In such a model for the RicMonk, the joint between $arm_1$ and ladder bar at $\mathbf{(y_{c}, z_{c})}$ is a revolute joint(dotted circle in the figure). In reality, the arm in contact with the support bar can move if enough force is applied to the arm. Making use of such a model reduces the number of constraints to be imposed, hence simplifying the problem. However, the use of such a fixed-base model has its disadvantages. $arm_1$ cannot let go of the bar, hence trajectories where that happens may not be optimized using this model.

The study on [AcroMonk](https://arxiv.org/abs/2305.08373), a two-linked brachiation robot, that is capable of continuously performing robust multiple brachiation maneuvers describes the use of a state machine. The state machine is an implementation that helps the robot to continually brachiate either forward or backward while taking the failure of brachiation into account. For instance, when the robot is not able to successfully catch the bar, when the robot loses all contact with the bar and falls on the ground, when the arm slips off the ladder bar, etc. Given a situation with the robot in any of these cases, the robot either tries to brachiate again using some recovery trajectories or shuts down the actuators to keep the user, the environment, and the robot itself safe. To have such an efficient state machine, the complex brachiation motion was broken down into four distinct simpler maneuvers(a.k.a. atomic behaviors), **ZB**, **ZF**, **BF**, and **FB**. These maneuvers translate respectively to 'zero to back', 'zero to front', 'back to front', and 'front to back'. The forward direction of motion is considered to be the direction in which the hook forms a concave shape. Each of these trajectories is illustrated in the following section. Such an implementation of a state machine enables the robot to perform continuous robust brachiation, even in the face of disturbances. In addition, it simplifies the task of optimizing trajectories. 

Formulation of the trajectory optimization problem is described in [here](formulation.md)

## Gripper Heuristics
Grasp constitutes the action of the robot swing arm hooking onto the surface of the ladder bar, whereas release constitutes the action of the robot swing arm letting go of the ladder bar. Trajectory optimization as [formulated](formulation.md) does not include grasp and release actions, as it does not consider the contact dynamics of the system. As a result, trajectories are optimized from the time instant after the swing arm would have performed the release action to the moment before the swing arm performs the grasp action. These trajectories are called atomic behaviors. Generally, the actions of the swing arm performing release and grasp require the application of a feed-forward torque to the robot's motors. In the case of RicMonk, the weight of the robot and the velocity of the approach of the swing arm toward the target ladder bar take care of the grasp. Hence, for RicMonk to perform the grasp action, no feed-forward torque is applied. However, to perform the release and grasp actions, direct feed-forward torque is applied to the actuators in appropriate directions. The directions and magnitude of the torques were calculated empirically. The following image describes the directions of force applied on the arms due to torque generated in respective motors to enable the robot to perform a release action.
<div align="center">
<img width="500" src="/hardware/imagesAndGifs/continuousBrachaition.png" />
</div>

To validate the release of the grasp, we monitor the velocity of the swing arm. When the swing arm is still grasping the ladder bar, its velocity remains minimal. We have set a threshold velocity of 1.2 rad/s, which indicates a successful release when the swing arm reaches this velocity. Our tests ensure that only minimal torque is applied to release the grasp from the ladder bar.


## Atomic Behaviors
The swing trajectories for these atomic behaviors are optimized and the animations are obtained as follows. 

<!-- <div align="center">
<img width="500" src="data/model/stateMachine.png" />
</div> -->

<div align="center">
<img width="250" src="hardware/imagesAndGifs/zb.gif" >
<img width="250" src="hardware/imagesAndGifs/zf.gif" >
<img width="250" src="hardware/imagesAndGifs/bf.gif" >
<img width="250" src="hardware/imagesAndGifs/fb.gif" >
</div>

<!---
<div align="center">
<img width="200" src="hardware/imagesAndGifs/ZB_ini_gif.gif" >
<img width="200" src="hardware/imagesAndGifs/ZF_ini_gif.gif" >
<img width="200" src="hardware/imagesAndGifs/BF_ini_gif.gif" >
<img width="200" src="hardware/imagesAndGifs/FB_ini_gif.gif" >
</div>
-->

## Operation
The folder **direct_collocation** contains files that provide the respective hyper-parameters for optimizing trajectories. The file **simulateTrajOpt.py** is to run for optimizing trajectories. Input the name of the trajectory (ZB, ZF, BF, FB) and the ladder bar distance (default = 0.34m). The optimized trajectories are saved in the folder with the name of the maneuver in [here](/data/trajectories/direct_collocation/), according to the maneuver.