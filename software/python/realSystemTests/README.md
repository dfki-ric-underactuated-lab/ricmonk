# Test files
This folder contains various files that can be used to test the functioning of RicMonk and all its components. The following provides information on the files. If some error arises like problem with saving or accessing files, try running the files as administrator by including `sudo` before the command used to run the file.

**Note:** With the current RicMonk, motor_9 (motor ID = 9) is attached to the left arm, and motor_8 (motor ID = 8) is attached to the right arm. The robot is said to be performing an odd brachiation if the left arm becomes the **stance arm** (the arm that holds onto the ladder bar as the rest of the robot swings) and the right arm becomes the **swing arm** (the arm the swings with the robot to help the robot brachiate forwards or backwards).

**Note:** Zero position is the position when the **stance arm** is vertically up and grasping the ladder bar, while rest of the robot hangs due to gravity. As zero position, the tail and the **swing arm** are also hanging vertically downwards.
<!---
**ADD A PICTURE OF THE ROBOT IN ZERO POSITION, maybe**
-->


## go-zero-pos.py
This file is used to command a single motor (motor ID and the bus the motor is connected to are variables in the code) to rotate from position to position. If the motor is already in some non-zero position, by changing the `final_pos` to zero and running this file, the motor returns to zero position. This file may be running using the following:
```bash
python3 -E go-zero-pos.py
```

## imu_validation_active_test.py
For testing the estimation of position and velocity of the unactuated joint using the IMU output, this code may be used. An assumption made is that an acroBot type set is used for this purpose, with the unactuated joint also attached to an actuator. The actuator attached to the unactuated joint is only used to validate the state estimation made using the IMU. The setup used is shown in the following image
Since this is an active test, the setup is made to follow a trajectory and the tracking of the same serves a reference for validating the state estimation method.
To run this file use:
```bash
sudo -E python3 imu_validation_active_test.py
```

<div align="center">
<img width="305" src="/hardware/imagesAndGifs/pasActTestSetup.jpeg" />
</div>

## imu_validation_passive_test.py
This test uses the same setup as that for the active test. However, given that this is a passive test, the motors are not given any trajectory to track. The setup needs to be manually moved, and comparision between the measured and estimated unactuated joint position and velocity may be used to validate the state estimation method.
To run the file use:
```bash
sudo -E python3 imu_validation_passive_test.py
```

## ladderReleaseLoop.py
This is an extensively written code to estimate the full state of the robot as it releases grasp from a ladder bar (front/back) for odd or even brachiations. These parameters need to set at the beginning of the code and the same continues as long as release tests are being performed.
Ensure that the robot is in the required zero position for the required type of brachiation (left arm is stance arm for odd brachiation, right arm is stance arm for even brachiation), before the code is run. Once ensured, run the code, set the robot with the required release position (swing arm on the back bar for back release test and swing arm on front bar for front release test). Reply accordingly to the question asked if the test need to be continued with. If the feed-forward torques were set as required before the code was run, then there may not be any requirement to change these values. However, if required these feed-forward torques may be varied for any iteration of the release tests. Answer accordingly to the question regarding changing these values. Subsequently, after the feed-forward the state information of the robot may be saved into an excel file based on the response to another question. Once this stage is completed, to continue with more release tests the robot may be set to release position again, and the question regarding more tests may be answered accordingly. 
Based on the information provided initially, the data from the tests are saved to respective excel files (evenBrachBackReleaseVelocities.xlsx, evenBrachFrontReleaseVelocities.xlsx, oddBrachBackReleaseVelocities.xlsx, oddBrachFrontReleaseVelocities.xlsx)
To run the cbashode:
```bash
sudo -E python3 ladderReleaseLoop.py
```

## motor_enable.py
To enable the motors, or stop them, or zero-offset both the motors, to instantly make sure that communication with the motors is intact, the following command may be used:
```bash
sudo -E python3 motor_enable.py
```

## single_odd_brachiation.py
As indicated by the name of this program, this program was used to test single odd brachiation maneuvers. To test the maneuver, firstly, the robot needs to be placed in the odd zero position. If testing the **BF** or **FB** trajectory, ensure that the proper feed-forward torque values are set in the function call of `kickback` function in the `brachiation` function. 
- Once set in the odd zero position (left arm hanging from support bar, right arm and tail vertically downwards), navigate to the current folder via terminal
- Ensure that the Makefile is set to run `single_odd_brachiation.py`.
- Turn ON the motors using the safety switch. Keep the safety switch close to you at all times
- Run the following snippet to enable the robot to perform the **BF** maneuver, and also informing that the motors are connected Bus 4 instead of 2. To enable the robot to perform other maneuvers, replace the `MANEUVER` field with appropriate maneuver name.
```bash
make BUS_NUMBER=4 MANEUVER=BF
```
- If the maneuver requested is either **BF** or **FB**, once the motors are calibrated, when the prompt asks if the robot is set at the required initial position, manually move the robot right arm to place the robot in a double support configuration. When done, input `y`.
- The robot would ideally perform the required maneuver
- If the robot behaves erratically or unsuccessfully performs a maneuver, immediately switch OFF the motors using the remote safety switch.

## read_imu.py
This program is run to test the functioning of the IMU. The configuration of the IMU may be modified as required in the code. Running this code continuously returns all the information from the IMU output continuously at a high frequency until the code is aborted. To run, use:
```bash
sudo -E python3 read_imu.py
```

## read_motor_data.py
This code runs to read and save the position and velocity values of a motor (motor ID and bus number are variables) over a certain period of time. The data is saved in a .csv and data is also presented in plots. To run, use:
```bash
sudo -E python read_motor_data.py
```

## tableTestMotorKickBack_try.py
Since release tests involve very high torques, the release (a.k.a kickback) functions may be tested, for a single motor, using this program, in such a way that robot arms are not connected. What is the purpose of this code when ladderReleaseLoop.py (arms dismantled) is present? Don't really know. Sorry. Run by using:
```bash
sudo -E python tableTestMotorKickBack_try.py
```

## test_multiple_brachiations.py
Already wrote about this [here](multipleBrachiationRealize.md)

## testOddEvenSwitches.py
This program is used to inspect if the changes between odd and even brachiation, if changes to be made as a result of the actuators completing a full rotation; are being handled properly during multiple brachiations (three motions is set to be a default, can be varied in line 208). During the code run, the motors don't supply any torque, and their position and velocities are read, and the state estimation is performed. The robot arms are to be manually passively moved to simulate the brachiation behaviour. The program prints a few lines to indicate start and end of a maneuver to help the user. Finally, a plot and .csv are output which can be used to validate the state estimation method. Before running the program, ensure that the robot is in odd zero position. Run the code using the following command and start moving the arms manually as required to imitate required maneuvers. Suggest to start with **ZF** trajectory, followed by successive **BF** maneuvers.
```bash
sudo -E python3 testOddEvenSwitches.py
```

## trajectory_replay.py
To test the trajectories without applying a controller, this program may be used, by running the following command:
```bash
sudo -E python3 trajectory_replay.py
```

## single_even_brachiation.py
This code is similar to the `single_odd_brachiation.py` code. Replace odd with even in the Makefile. Before running the code, place the robot in even zero position (right arm hanging from support bar, left arm and tail vertically downwards). Other steps are similar.

## two_motor_goto_pos.py
This program does exactly what the name of the program describes. It commands, once, two motors to go to a specified position. Run, using:
```bash
sudo -E python two_motor_goto_pos.py
```

## utils.py
This file contains all the required functions for the RicMonk to do what it does. 