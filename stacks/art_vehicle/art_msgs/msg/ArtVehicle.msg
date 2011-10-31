#  ART vehicle dimensions.
#  $Id$

#  This class encapsulates constants for the dimensions of the ART
#  autonomous vehicle.  All units are meters or radians, except where
#  noted.  This is not a published message, it defines multi-language
#  constants.

# ROS frame ID
string frame_id = "vehicle"

float32 length = 4.8                    # overall length
float32 width = 2.12                    # overall width
float32 height = 1.5                    # overall height (TBD)
float32 halflength = 2.4                # length / 2
float32 halfwidth = 1.06                # width / 2
float32 halfheight = 0.75               # height / 2
float32 wheelbase = 2.33918     	# wheelbase

# egocentric coordinates relative to vehicle origin at center of
# rear axle
float32 front_bumper_px = 3.5   	# (approximately)
float32 rear_bumper_px = -1.3           # front_bumper_px - length
float32 front_left_wheel_px = 2.33918   # wheelbase
float32 front_left_wheel_py = 2.4       # halfwidth
float32 front_right_wheel_px = 2.33918  # wheelbase
float32 front_right_wheel_py = -1.06    #-halfwidth
float32 rear_left_wheel_px = 0.0
float32 rear_left_wheel_py = 1.06       # halfwidth
float32 rear_right_wheel_px = 0.0
float32 rear_right_wheel_py = -1.06     #-halfwidth

# Player geometry, egocentric pose of robot base (the px really
# does need to be positive for some reason)
float32 geom_px = 1.1                   # front_bumper_px - halflength
float32 geom_py = 0.0
float32 geom_pa = 0.0

float32 velodyne_px = 0.393             # (approximately)
float32 velodyne_py = 0.278             # (approximately)
float32 velodyne_pz = 2.4               # (calibrated)
#float32 velodyne_yaw=-0.0343           # (before remounting)
float32 velodyne_yaw=-0.02155           # (approximately)
float32 velodyne_pitch=0.016353735091186868 # (calculated)
float32 velodyne_roll=0.0062133721370998124 # (calculated)

float32 front_SICK_px = 3.178
float32 front_SICK_py= 0.0		# (approximately)
float32 front_SICK_pz = 0.7
float32 front_SICK_roll = 0.0		# (approximately)
float32 front_SICK_pitch = 0.0		# (approximately)
float32 front_SICK_yaw = 0.027	        # (approximately)

float32 rear_SICK_px = -1.140
float32 rear_SICK_py = 0.0              # (approximately)
float32 rear_SICK_pz = 0.7
float32 rear_SICK_roll = 0.0		# (approximately)
float32 rear_SICK_pitch = 0.0		# (approximately)
float32 rear_SICK_yaw = 3.1415926535897931160  # (approximately PI)

float32 center_front_camera_px = 0.548     # velodyne_px + 0.155 (approx)
float32 center_front_camera_py = 0.278    # velodyne_py (approx)
float32 center_front_camera_pz = 2.184    # velodyne_pz-0.216 (approx)
float32 center_front_camera_yaw = -0.058  # (measured)
float32 center_front_camera_pitch = 0.0   # (assumed)
float32 center_front_camera_roll = 0.0    # (assumed)

float32 right_front_camera_px = 0.471    # velodyne_px + 0.078 (= 0.155 * cos 60 deg) (approx)
float32 right_front_camera_py = 0.144   # velodyne_py - 0.1342 (= 0.155 + sin 60 deg) (approx)
float32 right_front_camera_pz = 2.184   # velodyne_pz-0.216 (approx)
#float32 right_front_camera_yaw = -0.4974 # (approx -28.5 deg)
float32 right_front_camera_yaw = -1.0472 # (-60 deg, need to calibrate)
float32 right_front_camera_pitch = 0.0  # (assumed)
float32 right_front_camera_roll = 0.0   # (assumed)

float32 left_front_camera_px = 0.471     # velodyne_px + 0.078 (= 0.155 * cos 60 deg) (approx)
float32 left_front_camera_py = 0.412    # velodyne_py + 0.1342 (= 0.155 * sin 60 deg) (approx)
float32 left_front_camera_pz = 2.184    # velodyne_pz-0.216 (approx)
#float32 left_front_camera_yaw = 0.4974  # (approx +28.5 deg)
float32 left_front_camera_yaw = 1.0472  # (+60 deg, need to calibrate)
float32 left_front_camera_pitch = 0.0   # (assumed)
float32 left_front_camera_roll = 0.0    # (assumed)

# Compute vehicle turning radius.  This is the distance from the
# center of curvature to the vehicle origin in the middle of the
# rear axle.  The <art/steering.h> comments describe the steering
# geometry model.  Since max_steer_degrees is considerably less
# than 90 degrees, there is no problem taking its tangent.

float32 max_steer_degrees = 29.0        # maximum steering angle (degrees)
float32 max_steer_radians = 0.5061455   # maximum steering angle (radians)

# Due to limitations of the ROS message definition format, these
# values needed to be calculated by hand...

# ArtVehicle.wheelbase / math.tan(ArtVehicle.max_steer_radians)
float32 turn_radius = 4.2199922597674142

# math.sqrt(math.pow(ArtVehicle.wheelbase,2)
#           + math.pow(ArtVehicle.turn_radius + ArtVehicle.halfwidth,2))
float32 front_outer_wheel_turn_radius = 5.774952929297676

# math.sqrt(math.pow(ArtVehicle.wheelbase,2)
#           + math.pow(ArtVehicle.turn_radius - ArtVehicle.halfwidth,2))
float32 front_inner_wheel_turn_radius = 3.9315790916869484

# ArtVehicle.turn_radius + ArtVehicle.halfwidth
float32 rear_outer_wheel_turn_radius = 5.2799922597674147

# ArtVehicle.turn_radius - ArtVehicle.halfwidth
float32 rear_inner_wheel_turn_radius = 3.1599922597674142

# float32 front_outer_bumper_turn_radius = sqrtf(powf(front_bumper_px,2)+powf(turn_radius+halfwidth,2))
#  
# float32 front_inner_bumper_turn_radius = sqrtf(powf(front_bumper_px,2)+ powf(turn_radius-halfwidth,2))
#
# float32 rear_outer_bumper_turn_radius = sqrtf(powf(rear_bumper_px,2)+ powf(turn_radius+halfwidth,2))
#
# float32 rear_inner_bumper_turn_radius = sqrtf(powf(rear_bumper_px,2)+ powf(turn_radius-halfwidth,2))
