

## How to run

ROS2 version: Humble
Webots version: 2023a

First go to your development workspace and run:

    source /opt/ros/galactic/local_setup.bash
    colcon build
    source install/setup.bash

### RVIZ2 + Control

Start RVIZ2 by typing

    rviz2
    
and select the map visualization

You can control both the simulated and real crazyflie with twist messages:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard


### Real Crazyflie
You will need an [STEM ranging bundle](https://store.bitcraze.io/collections/bundles/products/stem-ranging-bundle) for this.

All these nodes will make the Crazyflie take off right away to height of 0.5 meters


#### Simple mapper

    ros2 launch crazyflie_ros2_simple_mapper simple_mapper_real_launch.py 
#### SlamToolbox
Not working ideally yet!

    ros2 launch crazyflie_ros2_slam slam_toolbox_real_launch.py 

#### NAV2
For now only with simple mapper

    ros2 launch crazyflie_ros2_navigation navigation_real_launch.py 

### Simulated Crazyflie

First install [webots 2022a](https://www.cyberbotics.com/)


This crazyflie webots controller uses the python bindings of the crazyflie firmware. Replace this line in crazyflie_ros2_simulation/crazyflie_ros2_simulation/crazyflie_webots_driver.py with the location of your [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) repo and do `make binding_pythons`:

    sys.path.append('/home/knmcguire/Development/bitcraze/c/crazyflie-firmware')
    import cffirmware
    
#### Simple mapper

    ros2 launch crazyflie_ros2_simple_mapper simple_mapper_simulation_launch.py 

#### SlamToolbox
Not working ideally yet!

    ros2 launch crazyflie_ros2_slam slam_toolbox_simulation_launch.py 

#### NAV2
With the slamtoolbox with use_scan_matching on False

    ros2 launch crazyflie_ros2_navigation navigation_simulation_launch.py 



