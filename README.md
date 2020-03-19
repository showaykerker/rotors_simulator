RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```

Difference from [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator)
-----------------------------------------------------------------------------------------
#### Enable RGB Camera
* Edit `rotors_simulator/rotors_description/urdf/component_snippets.xacro`
1. Change .so file of the plugin
```xml
  <robot xmlns:xacro="http://ros.org/wiki/xacro">
    ...
    <!-- Macro to add a camera. -->
    <xacro:macro name="camera_macro"
      params="namespace parent_link camera_suffix frame_rate
        horizontal_fov image_width image_height image_format min_distance
        max_distance noise_mean noise_stddev enable_visual *geometry *origin">
        ...
        <gazebo reference="${namespace}/camera_${camera_suffix}_link">
          <sensor type="camera" name="${namespace}_camera_${camera_suffix}">
            ...
            <plugin name="${namespace}_camera_${camera_suffix}_controller" filename="librotors_gazebo_noisydepth_plugin.so">
              ...
            </plugin>
          </sensor>
        </gazebo>
      </xacro:macro>
      ...
  </robot>

```
2. Change image format to R8G8B8.
```xml
  
  <robot xmlns:xacro="http://ros.org/wiki/xacro">
    ...
    
    <!-- This affects topic ${namespace}/hummingbird/vi_sensor/left/image_raw -->
    <!-- Macro to add a VI-sensor stereo camera. -->
    <xacro:macro name="vi_sensor_stereo_camera_macro"
       params="namespace parent_link frame_rate origin_offset_x baseline_y origin_offset_z max_range">
      <xacro:stereo_camera_macro
      ...
      image_format="R8G8B8"
      ...
      />
      ...
    </xacro:macro>

    ...

  </robot>
  
```

#### Change Image Size
* Edit `rotors_simulator/rotors_description/urdf/component_snippets.xacro`
* Simply replace all 480 -> 60, 752 -> 96, 640 -> 80'

#### Change Field of View (fov)
* Edit `rotors_simulator/rotors_description/urdf/component_snippets.xacro`
* Set horizontal_fov to 2.26893

#### Added `rpyt` to `header/frame_id` of the `Actuators` messages that are published by roll_pitch_yawrate_throttle_controller_node.


Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/kinetic/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/showaykerker/rotors_simulator/master/rotors_hil.rosinstall
 $ wstool merge rotors_hil.rosinstall
 $ wstool update
 ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. Should you receive a timeout error, try running gazebo by itself (e.g. roslaunch gazebo_ros empty_world.launch ) so it has sufficient time to actually download all of the models.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulation alongside ROS joystick driver and the RotorS joystick node:
```
$ roslaunch rotors_gazebo mav_with_joy.launch mav_name:=firefly world_name:=basic
```

Depending on the type of joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Usage with a keyboard

First, perform a one-time setup of virtual keyboard joystick as described here: https://github.com/ethz-asl/rotors_simulator/wiki/Setup-virtual-keyboard-joystick.

Launch the simulation with the keyboard interface using the following launch file:
```
$ roslaunch rotors_gazebo mav_with_keyboard.launch mav_name:=firefly world_name:=basic
```

If everything was setup correctly, an additional GUI should appear with bars indicating the current throttle, roll, pitch, and yaw inputs. While this window is active, the Arrows and W, A, S, D keys will generate virtual joystick inputs, which can then be processed by the RotorS joystick node in the same way as real joystick commands.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.
