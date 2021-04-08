^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cpswarm_sar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-29)
------------------
* Added: Global parameter RNG seed
* Added: FSM script for HITL
* Changed: Update launch files and events
* Changed: Use latest version of swarm functions and swarm behaviors libraries
* Changed: Simulated targets are handled by the swarm functions library
* Fixed: Make launch files work with latest PX4 flight controller version
* Contributors: Micha Sende

1.2.0 (2019-09-10)
------------------
* Added: Launch files for HITL
* Added: Launch files for new algorithms
* Added: Launch files for simulating more UAVs
* Added: RVIZ configuration
* Added: Simulation of targets
* Added: Simplified logging functionality
* Added: Scripts to run multiple simulation runs for performance evaluation
* Added: Scripts for performance analysis using bag files
* Changed: New launch file structure
* Changed: Restructured library package, move parts to other libraries
* Changed: UAVs yaw placement can be defined
* Changed: Important nodes are required in launch files
* Changed: No configuration files, all parameters are set hierarchically in launch files
* Fixed: MAVROS/PX4 UDP ports for simulation with more than 3 UAVs
* Contributors: Micha Sende, Omar Morando

1.1.0 (2018-11-12)
------------------
* Added: Parameter pos_type to choose between GPS and local positioning
* Changed: Restructured libraries
* Contributors: Micha Sende

1.0.0 (2018-10-30)
------------------
* Initial release
* Based on the SAR demo at the M18 review of the CPSwarm project
* Contributors: Micha Sende
