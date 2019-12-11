# define sensor
define lidar ranger
(
    sensor
    (
        range [0.0 3.0]
        fov 360
        samples 1440
    )
    color "black"
    size [0.05 0.05 0.1]
)

# define agent
define agent position
(
    size [0.1 0.1 1]
    origin [0 0 0 0]
    gui_nose 1
    drive "diff"
    color "random"
    lidar
    (
        pose [0 0 -0.5 0]
    )
)

# define floorplan
define floorplan model
(
    color "gray30"
    boundary 0
    gui_nose 0
    gui_grid 0
    gui_move 0
    gui_outline 0
    gripper_return 0
    fiducial_return 0
    laser_return 1
    pose [0 0 0 0]
)

# set the resolution of the underlying raytrace model in meters
resolution 0.05

# start simulation in paused mode (gui only)
paused 1

# run real time (gui only)
speedup 1

# simulation timestep in milliseconds
interval_sim 100

# set window properties (gui only)
window
(
    size [800 600] 
    rotate [0 0]
    scale 100
    show_data 1
)
