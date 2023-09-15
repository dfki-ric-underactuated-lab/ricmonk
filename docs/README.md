# RicMonk: basic information

Mechatronic design and integration of the RicMonk is addressed [here](../hardware/mechDesAndInt.md). 

## Raspberry Pi and its setup instructions
Raspberry Pi4([setup instructions](https://projects.raspberrypi.org/en/projects/raspberry-pi-setting-up)) is used as a proccessing unit for the RicMonk. Since the Raspberry Pi is housed in the RicMonk, communication to the Raspberry Pi is executed over WiFi using a server computer. An SSH connection is established over WiFi between the server and the Raspberry Pi. To do so, the following steps may be utilized:
- Setup a router
- Connect the Raspberry Pi to a keyboard, a mouse, and a monitor and then power it up
- Connect the Raspberry Pi to the router via WiFi
- Connect the server computer to the router via WiFi
- Get the IP address of the Raspberry Pi using the command `hostname -I` on the Raspberry Pi command line
-Enable SSH/VNC by using `sudo raspi-config` in the Raspberry Pi command line
- Given the username of the Raspberry Pi is `pi` and the IP address is `ipaddress` use the command `ssh pi@ipaddress` the command line of the server

Once all this is set up, there is no requirement to connect the Raspberry Pi to the peripherals anymore. It can be used via SSH.

All information required for pi3hat configuration and commanding motors using tview has been detailed [here](https://git.hb.dfki.de/underactuated-robotics/acroMonk/-/blob/master/docs/instructions/pi3hat_and_rpi.md)

# Moteus tools and Raspberry Pi

Welcome to the documentation for the Pi3hat and Raspberry Pi (RPi) integration. This guide is organized into the following sections:

- Installation
- Sending Commands Using `tview`
- Useful Terminal Commands
- IMU Sensor Data Readings

## Installation

To get started, you'll need to install the necessary dependencies. Follow these steps:

1. Install `moteus` and `moteus_pi3hat` by running the following commands:

```bash
pip3 install moteus
sudo apt install python3-pyside2* python3-serial python3-can python3-matplotlib python3-qtconsole
sudo pip3 install asyncqt importlib_metadata pyelftools
sudo pip3 install --no-deps moteus moteus_gui
sudo pip3 install moteus_pi3hat
```

If you encounter issues related to the `PySide2` dependency, execute these commands:

```bash
pip install PySide2
pip3 install PySide2
sudo apt install python3-pyside2* python3-serial python3-can python3-matplotlib python3-qtconsole
```

Afterward, try installing `moteus_pi3hat` again:

```bash
sudo pip3 install moteus_pi3hat
```

## Sending Commands Using `tview`

Depending on your specific configuration, which includes the bus connected (e.g., CAN) and motor ID, you can use the `tview` tool to connect to your motors. Execute the following command, replacing `BUS` and `MOTOR_ID` with your actual values:

```bash
sudo python3 -m moteus_gui.tview --pi3hat-cfg 'BUS=MOTOR_ID' -t MOTOR_ID
```

For instance, if your CAN connector is attached to `JC3` and the motor ID is `1`, use the following command:

```bash
sudo python3 -m moteus_gui.tview --pi3hat-cfg '3=1' -t 1
```

## Useful Terminal Commands

You can control and manage your motors using terminal commands. To stop or zero-offset a motor, execute these commands, ensuring you adjust the `--pi3hat-cfg` and `-t` options according to your setup. For example, if your CAN connector is connected to `JC3` and the motor ID is `1`, use the following commands:

- Zero-offset the motor:

```bash
sudo moteus_tool --zero-offset --pi3hat-cfg '3=1' -t 1
```

- Stop the motor:

```bash
sudo moteus_tool --stop --pi3hat-cfg '3=1' -t 1
```

### Important Note for Motor and Pi3hat Configuration

Please note that the motor ID for the system is currently set to `7`. Additionally, the only compatible buses with the Pi3hat are `J3` and `J5`. To connect to the motor with the current configuration, use one of the following commands:

```bash
sudo python3 -m moteus_gui.tview --pi3hat-cfg '3=7' -t 7
```

or

```bash
sudo python3 -m moteus_gui.tview --pi3hat-cfg '5=7' -t 7
```

## IMU Sensor Data Readings

### Reading IMU Data from Terminal

To access IMU data, you will need the `pi3hat_tools`. You can download a precompiled version of `pi3hat_tools` by following [this link](https://github.com/mjbots/pi3hat/releases/download/0.1-20210609/20210609-pi3hat_tools-d1e8aa529fe9aa62e6c0df19f10b83bd0e743273.tar.bz2).

**Note:** If your Raspberry Pi does not have an internet connection, you can download the file on your laptop and transfer it to the Raspberry Pi using the steps outlined [here](https://spellfoundry.com/docs/copying-files-to-and-from-raspberry-pi-and-mac/).

