
define sick_laser ranger
(
  #range_min 0.0        
  range_max 80.0
  fov 180               # should be 3.14159 radians, but it's not
  samples 181

  color "blue"
  size [ 0.14 0.14 0.2 ]
)

define 2d_velodyne ranger
(
  #range_min 0.0        
  range_max 80.0
  fov 360               # should be 2 pi radians, but it's not
  samples 360

  color "blue"
  size [ 0.14 0.14 0.2 ]
  origin [0.393 0.278 2.1 0.0]
)

define null_laser ranger
(
  range_min 0.0
  range_max 0.0
  fov 0.0
  samples 0

  color "purple"
  size [ 0 0 0 ]	
)

