# sar

This ROS package contains the all files (state machines, configuration, launch files) for the unmanned areal vehicles (UAVs) in the search and rescue (SAR) use case of the CPSwarm project.

## Setup
0. Requirements
   * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) (or newer)
   * [MAVROS](http://wiki.ros.org/mavros)
   * [PX4 flight controller](https://github.com/PX4/Firmware) (for simulations)
   * [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs) (for simulations)
   * [CPSwarm Communication Library](https://github.com/cpswarm/swarmio)
1. Create a [catkin](http://wiki.ros.org/catkin/workspaces) workspace
   ```
   mkdir ros_cpswarm
   cd ros_cpswarm
   mkdir src
   catkin init --workspace .
   catkin build
   ```
2. Clone this Git repositories
   ```
   cd src
   git clone https://github.com/cpswarm/complex_behaviors.git
   ```
3. Similarly, clone the Git repositories of required libraries
   * CPSwarm Swarm Library
     * [Swarm Behaviors](https://github.com/cpswarm/swarm_behaviors)
     * [Swarm Functions](https://github.com/cpswarm/swarm_functions)
   * CPSwarm Abstraction Library
     * [Hardware Functions](https://github.com/cpswarm/hardware_functions)
     * [Sensing and Actuation](https://github.com/cpswarm/sensing_actuation)
     * [Hardware Drivers](https://github.com/cpswarm/hardware_drivers)
   * [CPSwarm ROS Messages](https://github.com/cpswarm/cpswarm_msgs)

3. Prepare the environment by adding the following code to ~/.bashrc
   ```
   source /opt/ros/kinetic/setup.bash # use correct ROS version here
   export ROSCONSOLE_FORMAT='[${node}:${function}]: ${message}'
   export SVGA_VGPU10=0
   # setup cpswarm
   source ~/ros_cpswarm/devel/setup.bash
   # setup px4
   export PX4_HOME_LAT=12.3456 # replace GPS coordinates by the desired localization origin
   export PX4_HOME_LON=7.890
   source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/posix_sitl_default > /dev/null
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/cpswarm/src/Firmware:/home/cpswarm/src/Firmware/Tools/sitl_gazebo
   ```
4. Restart the terminal
5. Build the workspace
   ```
   cd ros_cpswarm
   catkin build
   ```

## Execution
The code can be executed either in simulation (SITL) or deployed on the UAVs (HITL).

### Simulation
For simulation with the Gazebo simulator, execute the launch file
```
roslaunch sar gazebo_<n>.launch
```
where <n> is the number of UAVs in the swarm. This will launch the Gazebo simulator (nodes `gazebo` and `gazebo_gui`) with <n> UAVs. For each UAV, it launches the launch file `uav_sitl.launch` that executes the following launch files in a distinct namespace `/cpswarm/$(arg vehicle)_$(arg id)` (see below for a description of the parameters).
* `$(find sar)/launch/px4.launch`
  The PX4 flight controller as SITL which will spawn a new UAV in the simulator.
* `$(find mavros)/launch/px4.launch`
  The MAVROS node which connects to the simulated PX4 flight controller.
* `$(find sar)/launch/uav_communication_library.launch`
  The CPSwarm communication library enabling communication between CPSs in the swarm.
* `$(find sar)/launch/uav_abstraction_library.launch`
  The CPSwarm abstraction library which launches the required nodes to access the hardware functionality.
* `$(find sar)/launch/uav_swarm_library.launch`
  The CPSwarm swarm algorithms to perform the SAR mission.
* `$(find sar)/launch/logging.launch`
  A node for logging information about the mission.

### Hardware
To run the code on deployed on real hardware, execute the launch file
```
roslaunch uav_hitl.launch
```
This will launch the following launch files in a distinct namespace `/cpswarm/$(arg vehicle)_$(arg id)` (see below for a description of the parameters).
* `$(find mavros)/launch/px4.launch`
  The MAVROS node which connects to the PX4 flight controller.
* `$(find sar)/launch/uav_communication_library.launch`
  The CPSwarm communication library enabling communication between CPSs in the swarm.
* `$(find sar)/launch/uav_abstraction_library.launch`
  The CPSwarm abstraction library which launches the required nodes to access the hardware functionality.
* `$(find sar)/launch/uav_swarm_library.launch`
  The CPSwarm swarm algorithms to perform the SAR mission.
* `$(find sar)/launch/logging.launch`
  A node for logging information about the mission.

### Launch File Parameters
Each launch file has a set of arguments which can be used to override the default values of the parameters. They are inherited through the hierarchy of the launch files.

#### gazebo_<n>.launch
* `gui` (bool, default: false)
  Whether to show the graphical user interface (GUI).
* `world` (string, default: $(find mavlink_sitl_gazebo)/worlds/empty.world)
  The world which defines the environment of the simulations.

#### uav_sitl.launch
* `id` (integer, default: 1)
  The identifier (ID) of the UAV in the swarm.
* `vehicle` (string, default: iris)
  The type of UAV that is simulated.
* `x` (real, default: 0)
  The starting position x-coordinate.
* `y` (real, default: 0)
  The starting position y-coordinate.
* `Y` (real, default: 0)
  The starting position yaw angle.
* `pos_type (string, default: local)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).
* `udp_port_obc` (integer, default: 14540 + id)
  The port number at which the simulated on board computer running MAVROS listens to incoming connections.
* `udp_port_fcu` (integer, default: 14550 + id)
  The port number at which the simulated flight controller PX4 listens for incoming connections from the on board computer.
* `udp_port_sim` (integer, default: 14560 + id)
  The port number which the simulated flight controller PX4 uses to communicate with the simulator.

#### uav_hitl.launch
* `id` (integer, default: 1)
  The identifier (ID) of the UAV in the swarm.
* `vehicle` (string, default: drone)
  The type of UAV that is simulated.
* `x` (real, default: 0)
  The x-coordinate measured from the origin of the relative localization system.
* `y` (real, default: 0)
  The y-coordinate measured from the origin of the relative localization system.
* `pos_type (string, default: local)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `output` (string, default: screen)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).
* `fcu_url` (string, default: /dev/ttyS2:57600)
  The path to the interface for communication with the flight controller.

### Parameter Files
In the `param` subdirectory there are two parameter files.

#### targets.yaml
This parameter file specifies the locations of the targets of the SAR mission during simulation.
* `targets_x` (real array)
  The x-coordinates of the target locations.
* `targets_y` (real array)
  The y-coordinates of the target locations.

#### uav_swarmros.cfg
The configuration file of the communication library. Refer to the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) for more information.
