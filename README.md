# cram_ros2
ROS2 Packages for the Cooperative Robotic Additive Manufacturing Project
 

## ROS2 Setup

The packages in this repository are compatible with ROS2 Humble and are best used with the Ubuntu 22.04 Linux distribution. For a smooth installation of ROS2 Humble, we recommend following the official guidelines provided at the [ROS2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html). For those utilizing Ubuntu 22.04, it is advised to opt for the Debian package installation method to ensure the best compatibility and performance.

To streamline your workflow and avoid the need to manually source the ROS2 Humble environment with every new terminal session, you can append the source command to your .bashrc file. This ensures the ROS2 environment is automatically initialized for you. Here's how you can do it:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
After executing this command, either restart your terminal for the changes to take effect or apply the changes immediately with:
```
source ~/.bashrc
```

## Universal Robots Dependency Installation

Install the Universal Robots ROS2 dependencies using:
```
sudo apt install ros-humble-ur
```
---
## Repository Setup

1. Create a workspace directory: `mkdir -p {workspace_name}/src`
2. Change directory to `/{workspace_name}/src`.
3. Clone the package into the `src` folder by cloning the repository
   ```
   git clone https://github.com/seanres/cram_ros2.git
   ```
7. Build the workspace by first changing the directory back to `../{workspace_name}` and then entering the following command:
    ```
    colcon build
    ```
    If this build fails, due to a "setup.py install deprecation warning" fix it by running `pip install setuptools==58.2.0`


In every terminal use the `source` command to source the overlay bash script: `source ${workspace_name}/install/setup.bash`

---

## Mock Hardware Setup

To work with the UR3 robot in simulation use these instructions.

In your **first terminal**, after sourcing the overlay, launch the driver with fake hardware flag using the following commands:
```
ros2 launch ur_robot_driver ur3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false
```
If you see the similar lines you are ready to move forward:
```
[INFO] [spawner-9]: process has finished cleanly [pid 65858]
[INFO] [spawner-4]: process has finished cleanly [pid 65848]
[INFO] [spawner-5]: process has finished cleanly [pid 65850]
```

---
In your **second terminal**, after sourcing the overlay, launch moveit with the following command:
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Once you see the message "You can start planning now!", it indicates that the MoveIt planning package has been successfully set up and is ready to be used. You can now proceed with your planning tasks in the simulation environment.


## Real Hardware Setup

To work with the real UR3 robot use these instructions.

- [] Test out the ur_description package and ur_moveit_package
- [] Write down IP configuration
- [] Write down launch commands

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.0.101 initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

