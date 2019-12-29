^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package complex_behaviors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2019-12-29)
------------------
* Changed: Create stack for complex_behaviors library
* Added: Global parameter RNG seed
* Changed: Update launch files and events
* Changed: Use latest version of swarm functions and swarm behaviors libraries
* Added to sar: FSM script for HITL
* Changed: Simulated targets are handled by the swarm functions library
* Fixed sar: Make launch files work with latest PX4 flight controller version
* Added to storage: FSM state machines for scouts and workers
* Added to storage: Parameters for number of scouts and workers
* Changed storage: Launch files support both HITL and SITL
* Changed storage: Log data to bag files
* Contributors: Micha Sende

1.2.0 (2019-09-10)
------------------
* First public release
* Added: Package storage
* Changed: Restructured library package, move parts to other libraries
* Added to sar: Launch files for HITL
* Added to sar: Launch files for new algorithms
* Added to sar: Launch files for simulating more UAVs
* Added to sar: RVIZ configuration
* Added to sar: Simulation of targets
* Added to sar: Simplified logging functionality
* Added to sar: Scripts to run multiple simulation runs for performance evaluation
* Added to sar: Scripts for performance analysis using bag files
* Changed sar: New launch file structure
* Changed sar: UAVs yaw placement can be defined
* Changed sar: Important nodes are required in launch files
* Changed sar: No configuration files, all parameters are set hierarchically in launch files
* Fixed sar: MAVROS/PX4 UDP ports for simulation with more than 3 UAVs


* Contributors: Micha Sende

1.1.0 (2018-11-12)
------------------
* Added to sar: Parameter pos_type to choose between GPS and local positioning
* Changed sar: Restructured libraries
* Contributors: Micha Sende

1.0.0 (2018-10-30)
------------------
* Initial release
* Based on the SAR demo at the M18 review of the CPSwarm project
* Contributors: Micha Sende
