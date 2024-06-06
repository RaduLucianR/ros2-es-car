# es-car with ROS2
This repository implements the ROS2 infrastructure necessary to control the RC car of the Electronics & Embedded Systems Division of KTH.
Before starting with ROS2, the wire connections must be in place. The wire connections between the "master" Raspberry Pi and the BeagleBone platforms can be seen in [this](https://github.com/RaduLucianR/ros2-es-car/wiki/Wire-connections) wiki page.

Currently (06-06-24), the repository has 2 packages:
1. **es_car** - the *main* package, with all the functionality
2. **es_car_inter** - the "interfaces" package, currently (06-06-24) only with custom messages. In the future, services and more messages can be added. The reason for having a separate package for interfaces is ROS2 itself, which prefers this structure, mainly because it's easier to import the interfaces in other packages.

### ROS2 architecture
![ros2 (1)](https://github.com/RaduLucianR/ros2-es-car/assets/57638808/061b6593-6808-475f-bfcd-43b5b3786b0a)


### Start-up
After the package installation the system may be started as follows.
1. Don't forget to compile and source the ROS2 workspace.
```
cd <ros2_workspace>
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

2. Start the ROS2 nodes for **steering** and **throttle** with the provided launch file:
```
ros2 launch es_car main_launch.py
```

3. In a different terminal, start the node for controlling the car with the keyboard by directly calling its executable:
```
ros2 run es_car wasd_control
```

4. Use the WASDXB keys to control the car. One press of each key has the following effect:
  - 'W' => increase the speed by +1 if speed > 138, otherwise set it to 138 (i.e. move slowly forwards)
  - 'S' => decrease the speed by -1 if speed < 110, otherwise set it to 110 (i.e. move slowly backwards) **if the car was in neutral beforehand**
  - 'A' => steer the wheels by 20 to the left
  - 'D' => steer the wheels by 20 to the right
  - 'S' => set the speed to 130 (neutral)
  - 'B' => set the speed to 80 (brake)

**Important note**: The driving backwards behavior is a bit inconsistent. What works is to follow this sequence of actions:
1. Brake, i.e. press 'B'
2. Set to neutral, i.e. press 'S'
3. Drive backwards, i.e. press 'X'
