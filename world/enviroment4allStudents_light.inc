# Defining a Simulated enviroment based on a Race enviroment

# the size of a pixel in Stage's (in methers)
resolution 0.01

# Including the model of the wireless repeaters
include "beacons_positions.inc"

# GUI options
gui_disable 1		#Necessary to show multiple robots
gui_interval 50	

# Configure GUI window 
window
( 
  	size [1120 898] 	#[1120 898] in pixels
  	rotate [ 0.0 0.0 ]
	#center [ 0.0 0.0]	# If this option is active, it's necessary ot activate the grid 
  	scale 37.00		# pixels per meter (controls the zoom)
	
	show_data 0              # Show datas like laserrange 1=on 0=off
)

#Define the floorplan model
define floorplan model
(
	# sombre, sensible, artistic
	color "grey30"	#Obstacles color

	# most maps will need a bounding box
	boundary 1

	gui_nose 0
	gui_grid 0		# Makes the grill more visible
	gui_move 0
	gui_outline 0
	gripper_return 0
	fiducial_return 0
	laser_return 1
)

# Define an enviroment bitmap
floorplan
( 
  	name "Race"
  	bitmap "bitmap/RaceMap_light.png"
	size [32.28 20.44 0.6]  #(1614px 1022px) * 0.02m/px [m_x m_y altitude_of_walls]
	pose [0 0 0 0]
)
