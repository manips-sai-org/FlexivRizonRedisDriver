# FlexivRizonRedisDriver for SAI
Redis driver compatible with SAI for the Flexiv Rizon robot

This program provides a wrapper around the Flexiv API to control a flexiv robot in torque mode. It reads torque commands from redis and publishes to redis the robot joint angles, joint velocities, sensed torques, mass matrix, gravity torques, coriolis torques.

It also implements torque saturation, joint position limit and joint velocity limit avoidance.

## Install

1. Follow the Flexiv Robot RDK Download and Install Instructions [here on the Flexiv website](https://www.flexiv.com/software/rdk/manual/index.html)

2. Clone this repo

3. As noted in the Flexiv Install instructions, you'll need to add an additional flag to the cmake command when building the driver in order to access the RDK Install Files. Replace "~/rdk_install" below with the absolute path to your RDK installation folder if necessary: 
```
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install
make -j4
```

## Usage

1. Prepare the robot for torque control (UPDATE with more precise instructions if needed)

2. Open a terminal and go to the FlexivRizonRedisDriver/redis_driver folder

3. Launch the redis server from the provided script
```
sh launch_redis_server.sh
```

4. Start the driver, with or without gripper depending on the hardware. The driver needs a config file as argument. The config file should be located in the folder config_folder.
```
./build/flexiv_rizon4_redis_driver_with_gripper <config_file_name>
```
or
```
./build/flexiv_rizon4_redis_driver <config_file_name>
```

## Config file and redis keys

The config file needs to be in the folder `config_folder`. It is an xml file with one `saiFlexivDriverConfig` tag containing the following attributes:

<saiFlexivDriverConfig robotName="4s-Oberon" serialNumber="Rizon4s_062232" robotType="Rizon4s" redisPrefix="sai"/>

- redisPrefix: the prefix for the redis keys
- robotName: the name of the robot (for constructing the redis keys)
- serialNumber: the serial number of the robot to control (for example "Rizon4s_062232")
- robotType: can be either "Rizon4" or "Rizon4s"

#### The robot driver will listen to the following redis key:

`<prefix>::commands::<robot_name>::control_torques`: this is the key sending the command torques to the robot. The robot performs its own gravity compensation so the sent command torques should not contain the gravity compensation torques.

#### The robot driver will publish the following keys:

`<prefix>::sensors::<robot_name>::joint_positions`: the position of the robot joints

`<prefix>::sensors::<robot_name>::joint_velocities`: the velocities of the robot joints

`<prefix>::sensors::<robot_name>::joint_torques`: the sensed joint torques (from robot torque sensors)

`<prefix>::sensors::<robot_name>::model::mass_matrix`: the robot joint space mass matrix

`<prefix>::sensors::<robot_name>::model::robot_gravity`: the joint gravity torques

`<prefix>::sensors::<robot_name>::model::coriolis`: the joint coriolis and centrifugal forces vector

`<prefix>::sensors::<robot_name>::safety_controller::safety_torques`: The torques computes by the integrated safety controller

`<prefix>::sensors::<robot_name>::safety_controller::constraint_nullspace`: The nullspace of the safety controller constraint task

`<prefix>::sensors::<robot_name>::safety_controller::sent_torques`: The torques actually sent to the robot. This will be equal to the command torques when no joint position, velocity or torque limit is triggered.

#### Additionally if the robot is the Rizon 4s model, the driver will directly publish the data from the integrated wrist force-torque sensor using the following keys:

`<prefix>::sensors::<robot_name>::ft_sensor::force`: The forces felt by the robot at the wrist in the local force sensor frame.

`<prefix>::sensors::<robot_name>::ft_sensor::force`: The moments felt by the robot at the wrist in the local force sensor frame.

#### The gripper driver additionally reads the following keys:

`<prefix>::<robot_name>::gripper::mode`: The mode for the gripper. Can be "o" to open the gripper or "g" to grasp (close the gripper)
