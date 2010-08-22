
define sick_laser laser
(
  #range_min 0.0        
  range_max 80.0
  fov 180               # should be 3.14159 radians, but it's not
  samples 181

  color "blue"
  size [ 0.14 0.14 0.2 ]
)

define null_laser laser
(
  range_min 0.0
  range_max 0.0
  fov 0.0
  samples 0

  color "purple"
  size [ 0 0 0 ]	
)

