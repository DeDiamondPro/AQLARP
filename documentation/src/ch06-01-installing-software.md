# Installing software
In this chapter we will go over installing and setting up all software required for AQLARP to work. You should already have AQLARP's GitHub repo cloned in your home directory as specified in [chapter 4.1](/ch04-01-raspberry-pi-setup.html#clone-the-github-repo)
## Installing ros2
AQLARP is built using ros2 iron, it might work with later versions but this is untested. To install ros2 iron you can follow [their installation instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html). After installing ros2 you must also install colcon, this is ros2's build system. To do this run the command below.
```console
$ sudo apt install python3-colcon-common-extensions
```
## Sourcing ros2 automatically
To use ros2 you must source it every time, this can be annoying. To do this automatically run the following command.
```console
nano ~/.bashrc
```
Then add these 2 lines at the end of the file.
```bash
source /opt/ros/iron/setup.bash
source ~/new_ws/install/setup.bash
```
Then press `Ctrl + O` to save and `Ctrl + X` to exit. To make this apply you should log out of the Raspberry Pi and start a new shell session.
## Installing dependencies
To install dependencies first go into the workspace directory, to do this run the following command.
```console
$ cd ~/AQLARP/ros2_ws
```
Then run the following command to install all required dependencies using rosdep.
```console
$ rosdep install --from-paths src -y --ignore-src
```
Then additionally you have to install pygame using pip, since the version rosdep uses is severally outdated. To do this run the following command.
```console
$ python3 -m pip install pygame
```

## Building the code
You should be in the workspace directory as specified in the previous step. Then to build the code run the following command and wait for it to complete
```console
$ colcon build
```
## Calibrating the servos
Since the servos aren't exactly at 90° they have to be calibrated. To run the calibration script run the following command.
```console
$ ros2 run aqlarp_motors calibration
```
Then to use the calibration script use the left and right arrow keys to increase or decrease the angle offset and use the up and down arrow keys to switch servo. You should try to set all servos as close to 90 degrees as possible.
## Running the code
To run all code of AQLARP, and thus start the robot, run the following command.
```console
$ ros2 launch aqlarp_main aqlarp.launch.py
```
To control the robot you will have to connect a ps4 controller, you can either do this by using a cable and connecting it using one of the USB ports, or alternatively you can use the instructions in the next part to connect the ps4 controller wirelessly.

The controls for AQLARP are as follows:
- Playstation button: start and stop the servos
- Left Joystick: Make the robot strafe in the direction you specify
- Right Joystick: Make the robot turn by moving the joystick left and right.
## Connecting ps4 controller wirelessly
Open a command prompt on the raspberry pi (with ssh) and run the following command to get into the Bluetooth command line.
```console
$ bluetoothctl
```
Then start scanning for Bluetooth devices with the following command.
```bash
scan on
```
Now press and hold the PlayStation and share button on your controller for a few seconds until the lightbar starts flashing. Then look at the output in your terminal for something like this.
```
Device A4:AE:11:41:FD:98 Wireless Controller
```
Copy the MAC address (the numbers in the center) and run the next command to pair with the controller. Make sure the lightbar is still flashing, if it isn't flashing anymore hold down the PlayStation and share button again.
```
pair <MAC address>
```
Then finally to make it so the controller can connect to AQLARP by just pressing the PlayStation button, run
```
trust <MAC address>
```
Now type `exit` to exit the Bluetooth terminal. If you want to disconnect the controller, hold down the PlayStation button for 10 seconds. To reconnect the controller, press the PlayStation button once.

## Making the code run on startup
If you want to make AQLARP's code run on startup so you can just connect the controller and use it, you will have to create a file called `aqlarp.service` at `/etc/systemd/system/`.
To do this run the following command.
```console
$ sudo nano /etc/systemd/system/aqlarp.service
```
Then paste the following content into it.
```toml
[Unit]
Description="AQLARP daemon"

[Service]
User=AQLARP
Restart=always
RestartSec=1

Environment="LD_LIBRARY_PATH=/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_interfaces/lib:/opt/ros/iron/lib/aarch64-linux-gnu:/opt/ros/iron/lib"
Environment="PYTHONPATH=/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_sensors/lib/python3.10/site-packages:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_motors/lib/python3.10/site-packages:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_main/lib/python3.10/site-packages:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_interfaces/lib/python3.10/site-packages:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_input/lib/python3.10/site-packages:/opt/ros/iron/lib/python3.10/site-packages"
Environment="AMENT_PREFIX_PATH=/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_sensors:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_motors:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_main:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_interfaces:/home/AQLARP/AQLARP/ros2_ws/install/aqlarp_input:/opt/ros/iron"

ExecStart=/opt/ros/iron/bin/ros2 launch aqlarp_main aqlarp.launch.py

[Install]
WantedBy=multi-user.target
```
If you use a different user then AQLARP or have cloned the GitHub repo in a different location, you will have to edit this file accordingly. To save the file press `Ctrl + O` and then press `Ctrl + X` to save.

Then finally to enable the service run the following command.
```console
$ sudo systemctl enable --now aqlarp.service
```