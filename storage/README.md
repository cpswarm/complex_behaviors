# storage

This ROS package contains the all files (state machines, configuration, launch files) for the unmanned ground vehicles (UGVs) in the logistics box storage use case of the CPSwarm project.


## Setup
0. Requirements
   * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) (or newer)
   * [MAVROS](http://wiki.ros.org/mavros)
   * Either one simulator
     * [Stage](http://wiki.ros.org/stage)
     * [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)
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
   ```
4. Restart the terminal
5. Build the workspace
   ```
   cd ros_cpswarm
   catkin build
   ```

## Execution
The code can be executed either in simulation (SITL) or deployed on the UGVs (HITL).

### Simulation
For simulation with the Stage simulator, execute the launch file
```
roslaunch storage stage_<n>.launch
```
where <n> is the number of UGVs in the swarm. This will launch the Stage simulator with <n> UGVs. For each UGV, it launches the launch file `ugv_sitl.launch` that executes the following launch files in a distinct namespace `/cpswarm/$(arg vehicle)_$(arg id)` (see below for a description of the parameters).
* `$(find storage)/launch/ugv_communication_library.launch`
  The CPSwarm communication library enabling communication between CPSs in the swarm.
* `$(find storage)/launch/ugv_abstraction_library.launch`
  The CPSwarm abstraction library which launches the required nodes to access the hardware functionality.
* `$(find storage)/launch/ugv_swarm_library.launch`
  The CPSwarm swarm algorithms to perform the logistics mission.
* `$(find storage)/launch/logging.launch`
  A node for logging information about the mission.

### Hardware
To run the code on deployed on real hardware, execute the launch file
```
roslaunch ugv_hitl.launch
```
This will launch the following launch files in a distinct namespace `/cpswarm/$(arg vehicle)_$(arg id)` (see below for a description of the parameters).
* `$(find mavros)/launch/px4.launch`
  The MAVROS node which connects to the PX4 flight controller.
* `$(find storage)/launch/ugv_communication_library.launch`
  The CPSwarm communication library enabling communication between CPSs in the swarm.
* `$(find storage)/launch/ugv_abstraction_library.launch`
  The CPSwarm abstraction library which launches the required nodes to access the hardware functionality.
* `$(find storage)/launch/ugv_swarm_library.launch`
  The CPSwarm swarm algorithms to perform the logistics mission.
* `$(find storage)/launch/logging.launch`
  A node for logging information about the mission.

### Launch File Parameters
Each launch file has a set of arguments which can be used to override the default values of the parameters. They are inherited through the hierarchy of the launch files.

#### stage_<n>.launch
* `gui` (bool, default: false)
  Whether to show the graphical user interface (GUI).
* `world` (string, default: empty)
  The world which defines the environment of the simulations.

#### ugv_sitl.launch
* `id` (integer, default: 1)
  The identifier (ID) of the UGV in the swarm.
* `vehicle` (string, default: iris)
  The type of UGV that is simulated.
* `x` (real, default: 0)
  The starting position x-coordinate.
* `y` (real, default: 0)
  The starting position y-coordinate.
* `world` (string, default: $(find mavlink_sitl_gazebo)/worlds/empty.world)
  The world which defines the environment of the simulations.
* `pos_type (string, default: local)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `output` (string, default: screen)

#### ugv_hitl.launch
TODO

### Parameter Files
In the `param` subdirectory there are two parameter files.

#### boxes.yaml
This parameter file specifies the locations of the boxes of the logistics mission during simulation.
* `boxes_x` (real array)
  The x-coordinates of the box locations.
* `boxes_y` (real array)
  The y-coordinates of the box locations.

#### ugv_swarmros.cfg
The configuration file of the communication library. Refer to the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio) for more information.
