define kinect ranger
(
  sensor
  (
    range [0.0 10.0]
    fov 190
    samples 800
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)
define block model
(
  size [0.25 0.2500 0.100]
  gui_nose 0
)
define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.25 0.25 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"
  
  kinect(pose [ 0.0 0.0 -0.202 0.0 ])
)
