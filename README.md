# RicMonk
</div>


<div align="center">
<img width="605" src="/hardware/imagesAndGifs/6ForBrachGif.gif" />
</div>

## Description
The RicMonk is the first three-link underactuated brachiation robot with passive hook-shaped grippers to be able to perform multiple brachiation maneuvers continuously. Though it has three generalized coordinates it has only two actuators, making it underactuated. The RicMonk is a great test bench for delving into Underactuated Robotics subjects like trajectory optmization and stabilizaition.

## Documentation
The following provide theoretical information regarding the RicMonk:
- [System dynamics](/hardware/dynamic.md)
- [Mechatronic design and integration](/hardware/mechDesAndInt.md)
- [Trajectory optimization](/software/ricMonkTrajOptStab/software/python/simulation/behavior_generation/trajectory_optimization/trajOpt.md)
- [Trajectory stabilization](/software/ricMonkTrajOptStab/software/python/simulation/behavior_control/README.md)

For operating the RicMonk, the following may serve as reference:
- [Basic information](/docs/README.md)
- [Brachiaiton realization](/software/realSystemTests/multipleBrachiationRealize.md)
- [Other tests](/software/realSystemTests/README.md)


## Safety Notes #

When working with a real system be careful and mind the following safety measures:

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, currently limited supplies, and mechanical design.

* The robot must be placed in a ladder bar cage and kept at least one-meter distance from the acromonk in case of operation.

* Make sure you have access to an emergency stop while doing experiments. Be extra careful while operating in the pure torque control loop.

* The robot is equipped with an onboard Lithium Polymer battery and needs proper care and attention. Make sure that you have all the necessary information for the LiPo batteries.

