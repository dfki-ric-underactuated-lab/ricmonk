# Realizing Multiple brachiation on the RicMonk

## Forward brachiation
The following steps help in realizing the multiple brachiation motion, in the forward direction, on the RicMonk:
- Trajectories stabilized are stored in **../data/trajectories/closed_loop** with respect to the current file
- The program to be run for executing multiple brachiaitions in the forward direction is **test_multiple_brachiations.py**
- Ensure that RicMonk is on the ladder, with left arm on the ladder, while right arm and the tail are vertical
- Ensure that all the electrical connections are properly made, and that the motors are turned ON using the transmitter
- Navigate to the folder of the current file(the folder that also contains **test_multiple_brachiations.py**) on the Raspberry Pi terminal
- Open the **Makefile** and ensure that `FILE` is set to `BF_TVLQR`, `TEST` set to `tvlqr`(only TVLQR is currently able to track all trajectories on the real system) and `FILE_RECOVERY` is set to 'ZF_tvlqr.csv'. Ensure that the file to be run in the make file is 'test_multiple_brachiation'. Save and close the make file
- On the terminal run `make NBRACH=3`(for three brachiation maneuvers)
- Input 'y' until the question about maneuver is asked
- For the maneuver, first input 'ZF' and press enter. If everything was set properly, the RicMonk exceutes a **ZF** maneuver and brachiates forward. If the robot is not able to perform the maneuver properly, abort the program and try again. If it does not work, trajectory optimization and stabilization need to be performed again as required.  
- When the question comes up again, input 'BF' only if the robot has successfully performed the **ZF** maneuver. The robot should perform another forward brachiating maneuver
- Perform the previous step until the robot has completed the required number of brachiation maneuvers.


- Information regarding testing single brachiation maneuvers and other tests are provided in [here](README.md)