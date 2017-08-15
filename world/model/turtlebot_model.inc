# Define a turtlebot model

# Define the ranger of the robot
define utm_30lx_ranger ranger(
	sensor(
		range [0.15 6.0]
		fov 360.0 	# Field of view
		samples 360	# Step angle 0.25º (should be 1080)
		color "green"
	)
)

# Define the multiple platforms of the robot
define platform_turtle model
(
	block(
		points 6
		point[0] [ 0.13748 0.079375]
		point[1] [ 0.13748 -0.079375]
		point[2] [ 0 -0.17]
		point[3] [ -0.13748 -0.079375]
		point[4] [ -0.13748 0.079375]
		point[5] [ 0 0.17]
		z [0.001 0.001]
	)
	color "gray20"
)

# Define the separators to sustent the platforms
define separator_turtle model
(
	size [0.01 0.01 0.331]
	color "gray80"
)

# Define the battery box model for the laser
define laser_battery_box model(
	#Box of the battery for the laser
	size [0.19 0.11 0.06]
	color "gray80"
)

# Define the model of the laser
define laser_representation model(
	size [0.06 0.06 0.09]
	color "black"
)

#Define a TurtleBot2 robot
define turtlebot2 position
(
	color "red"				# Default color
	drive "diff"				# Differential steering model
	gui_nose 1				# Draw a nose on the robot so we can see which way it points
	obstacle_return 1		        # Can hit things (like other robots)
	laser_return 1		         	# Reflects laser beams

	localization "odom"
	localization_origin [ 0.0 0.0 0 0.0 ]	# Starting point for odometry
	#odom_error [ 0 0 0 0 ]
	odom_error [ 0.01 0.01 0 0.01 ]	# Odometry error or slip in X, Y and Theta
                                         	# (Uniform random distribution)
	# four DOF kinematics limits
	# [xmin xmax ymin ymax zmin zmax amin amax]
	velocity_bounds [-0.65 0.65 0 0 0 0 -90.0 90.0]
	acceleration_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0]	#TODO Check this param

	# Actual size x[m] y[m] z[m]
	size [0.354 0.354 0.089] 
	
	# Center of the representation
	origin [0.000 0.000 0.000 0.000]        
	
	# Estimated mass in Kg
	mass 10.0 		              
	
	#ranger_return 1.000		        # Reflects sonar beams	

	# Roomba
	block(
		points 12
		point[0] [ 0.0885 0.15329]
		point[1] [ 0.15329 0.0885]
		point[2] [ 0.177 0]
		point[3] [ 0.15329 -0.0885]
		point[4] [ 0.0885 -0.15329]
		point[5] [ 0 -0.177]
		point[6] [ -0.0885 -0.15329]
		point[7] [ -0.15329 -0.0885]
		point[8] [ -0.177 0]
		point[9] [ -0.15329 0.0885]
		point[10] [ -0.0885 0.15329]
		point[11] [ 0 0.177]
		z [0 0.089]
	)

	# Platforms of the robot
	platform_turtle(
		# Top platform
		pose [0 0 0 0]
		size [0.316 0.316 0.331] # Real altitude -0.089
	)
	platform_turtle(
		# Middle platform
		pose [0 0 0 0]
		size [0.316 0.316 0.119] # Real altitude -0.089
	)
	platform_turtle(
		# Botton platform
		pose [0 0 0 0]
		size [0.316 0.316 0.066] # Real altitude -0.089
	)

	# Separators to keep the platforms in their positions
	separator_turtle(
		# Front-Left separator
		pose [0.05 0.12 0.0 0]
	)
	separator_turtle(
		# Back-Left separator
		pose [-0.05 0.12 0 0]
	)
	separator_turtle(
		# Front-Right separator
		pose [0.05 -0.12 0 0]
	)
	separator_turtle(
		# Back-Right separator
		pose [-0.05 -0.12 0 0]
	)

	# Laser representation
	laser_battery_box(
		pose [0.05 0 0.331 0]
	)
	laser_representation(
		pose [0.0 0.0 0.391 0]
	)

	utm_30lx_ranger(
		pose [ 0.0 0.0 -0.05 0] # x[m] y[m] z[m] (real altitude) phi[º]  (old 0 0 0 0.451 0)
		#old pose 0.0 0.0 0.451 0
		#ctrl "lasernoise"			#Adding noise to the laser
	)
)




