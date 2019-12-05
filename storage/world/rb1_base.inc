include "laser.inc"


define elevator gripper
(
  color "black"
  paddle_state [ "open" "down" ]
  autosnatch 0
  obstacle_return 0
)


define rb1_base position
(

  size [0.45 0.45 0.3]

  block
  (
    points 8
    point[0] [0.375 0]
    point[1] [0.5 0.125]
    point[2] [0.5 0.375]
    point[3] [0.375 0.5]
    point[4] [0.125 0.5]
    point[5] [0 0.375]
    point[6] [0 0.125]
    point[7] [0.125 0]
    z [0 0.25]
  )

  origin [0.0 0 0 0]
  gui_nose 1
  drive "diff"
  localization "odom"
  odom_error [0.00 0.00 0.00 0.00]
  
  laser(pose [ 0.20 0.000 -0.30 0.000 ])
  elevator(pose [0.01 0 0 0 ])
)