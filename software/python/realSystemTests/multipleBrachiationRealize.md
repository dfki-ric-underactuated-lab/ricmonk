# Realizing Multiple brachiation on the RicMonk

## Forward brachiation
The following steps help in realizing the multiple brachiation motion, in the forward direction, on the RicMonk:
- Trajectories stabilized are stored in **../data/trajectories/closed_loop** with respect to the current file
- The program to be run for executing multiple brachiaitions in the forward direction is **test_multiple_brachiations.py**
- Ensure that RicMonk is on the ladder, with left arm on the ladder, while right arm and the tail are vertical
- Ensure that all the electrical connections are properly made, and that the motors are turned ON using the transmitter
- Via terminal, navigate to the folder of the current file(the folder that also contains **test_multiple_brachiations.py**) on the Raspberry Pi terminal
- Open the **Makefile** and ensure that the python code set to run is `test_multiple_brachiations.py`.
- Ensure that all parameters are properly set.
- Keep the safety switch close to you at all times
- If the parameters in the Makefile were not set right, the information can also be given in the command line as follows. The following enables RicMonk to perform three (change `NBRACH` value in command line for more number) multiple brachiation maneuvers (one **ZB**, followed by two **FB**) in the backward direction (`DIRECTION=-1`). It also changes the `BUS_NUMBER` parameter to 3 instead of 2. (To enable the robot to perform multiple forward brachiation maneuvers, `DIRECTION` must be set to 1).
```bash
make NBRACH=3 BUS_NUMBER=3 DIRECTION=-1
```
- If the robot behaves erratically or unsuccessfully performs a maneuver, immediately switch OFF the motors using the remote safety switch.


- Information regarding testing single brachiation maneuvers and other tests are provided in [here](README.md)