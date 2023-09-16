[![DOI](https://zenodo.org/badge/689729904.svg)](https://zenodo.org/badge/latestdoi/689729904)

<div align="center">

# RicMonk: A Three-Link Brachiation Robot with Passive Grippers

</div>


<div align="center">
<img width="605" src="hardware/imagesAndGifs/ricmonk_bb.gif" />
</div>

## Description
This project offers an open-source and low-cost kit to test control algorithms for underactuated robots. 
It introduces a three-link underactuated brachiation robot called **RicMonk** capable of bidirectional brachiation maneuver with passive grippers. 
RicMonk has two Quasi-Direct-Drives (QDD) that allow dynamic and agile locomotion. 
This project offers different control methods for trajectory stabilization which can be studied using the kit. Additionally, it provides a list of components, discusses best practices for implementation, and presents results from experiments with the simulator and the real system. This repository describes the hardware (CAD, Bill Of Materials (BOM), etc.) required to build the physical system and provides the software (URDF models, simulation, and controllers) to control it.


## Research Paper

The work is submitted to the IEEE International Conference on Robotics and Automation (ICRA 2024) and is currently under review. 

## Documentation

The following provides theoretical information regarding the RicMonk:
- [System dynamics](/hardware/dynamic.md)
- [Mechatronic design and integration](/hardware/mechDesAndInt.md)
- [Trajectory optimization](/software/python/simulation/behavior_generation/trajectory_optimization/README.md)
- [Trajectory stabilization](/software/python/simulation/behavior_control/README.md)
- [Bill Of Materials BOM](/hardware/bills-of-materials.md)


For operating the RicMonk, the following may serve as a reference:
- [Basic information](/docs/README.md)
- [Brachiaiton realization](/software/python/realSystemTests/multipleBrachiationRealize.md)
- [Software Guide](/software/python/README.md)

The CAD file is also provided on grabCAD.com. You can use the 3D viewer from their website to display the 3D model directly within your browser.
 - [Display RicMonk in 3D](https://grabcad.com/library/ricmonk-a-three-link-underactuated-brachiation-robot-with-passive-grippers-1)
## Authors #

* [Shivesh Kumar](https://robotik.dfki-bremen.de/en/about-us/staff/person/shku02) (Team Leader - Mechanics and Control)
* [Mahdi Javadi](https://robotik.dfki-bremen.de/en/about-us/staff/person/maja04) (Project Supervisor, Hardware and Software Concepts, Trajectory Optimization and Stabilization)
* Shourie S. Grama (Hardware and Software Maintainer, Design Optimization, Trajectory Optimization and Stabilization)
* [Hossein Zamani Boroujeni](https://robotik.dfki-bremen.de/en/about-us/staff/person/hoza01) (Tail Design and Gripper Design Improvement. The credit for original arm and gripper design goes to [Daniel Pizzutilo](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/dapi01) in [AcroMonk](https://github.com/dfki-ric-underactuated-lab/acromonk))

Feel free to contact us if you have questions about the test bench. Enjoy!

## Contributing

1. Fork it (<https://github.com/yourname/yourproject/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request


## Safety Notes #

When working with a real system be careful and mind the following safety measures:

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force, and speed using configurable parameters, currently limited supplies, and mechanical design.

* The robot must be placed in a ladder bar cage and kept at least one distance from the acromonk in case of operation.

* Make sure you have access to an emergency stop while doing experiments. Be extra careful while operating in the pure torque control loop.

* The robot is equipped with an onboard Lithium Polymer battery and needs proper care and attention. Make sure that you have all the necessary information for the LiPo batteries.



## Acknowledgments #
This work has been supported by the M-RoCK (FKZ01IW21002) and VeryHuman (FKZ01IW20004) projects funded by the German Aerospace Center (DLR) with federal funds from the Federal Ministry of Education and Research (BMBF) and is additionally supported with project funds from the federal state of Bremen for setting up the Underactuated Robotics Lab (201-342-04-2/2021-4-1).

<div align="center">
<img width="500" src="hardware/imagesAndGifs/Logo_Underactuated_Lab.gif" />
</div>

## License

This work has been released under the BSD 3-Clause License. Details and terms of use are specified in the LICENSE file within this repository. Note that we do not publish third-party software, hence software packages from other developers are released under their very own terms and conditions, e.g. Stable Baselines (MIT License) and Tensorflow (Apache License v2.0). If you install third-party software packages along with this repo ensure that you follow each license agreement.

## Citation

1. Grama S., Javadi M., Kumar S., Zamani H., Kirchner F., (2024). RicMonk: A Three-Link Brachiation Robot with Passive Grippers for Energy-efficient Brachiation. Submitted in: IEEE International Conference on Robotics and Automation (ICRA).
```
@article{2024_ICRA_ricmonk,
journal={RicMonk: A Three-Link Brachiation Robot with Passive Grippers for Energy-efficient Brachiation}, 
author={Grama, Shourie S and Javadi, Mahdi and Kumar, Shivesh and Zamani Boroujeni, Hossein and Kirchner, Frank},
note={submitted and it is currently under review}}
```

