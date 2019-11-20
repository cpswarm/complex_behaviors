define cart position
(
  size [0.8 0.8 0.45]
  #gui_nose 0
  obstacle_return 0
  ranger_return 8000.0
  color "gray"
  gripper_return 1
  name "cart"

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
    z [0 0.075]
  )
block
  (
    points 8
    point[0] [5.375 5]
    point[1] [5.5 5.125]
    point[2] [5.5 5.375]
    point[3] [5.375 5.5]
    point[4] [5.125 5.5]
    point[5] [5 5.375]
    point[6] [5 5.125]
    point[7] [5.125 5]
    z [0 0.075]
  )
block
  (
    points 8
    point[0] [5.375 0]
    point[1] [5.5 0.125]
    point[2] [5.5 0.375]
    point[3] [5.375 0.5]
    point[4] [5.125 0.5]
    point[5] [5 0.375]
    point[6] [5 0.125]
    point[7] [5.125 0]
    z [0 0.075]
  )
block
  (
    points 8
    point[0] [0.375 5]
    point[1] [0.5 5.125]
    point[2] [0.5 5.375]
    point[3] [0.375 5.5]
    point[4] [0.125 5.5]
    point[5] [0 5.375]
    point[6] [0 5.125]
    point[7] [0.125 5]
    z [0 0.075]
  )

block
  (
    points 4
    point[0] [0 0]
    point[1] [0 5.5]
    point[2] [5.5 5.5]
    point[3] [5.5 0]
    z [0.075 0.1]
  )


mass 10.0

)
