The dataset.txt file contains the data that come from the sensor of a real robot.
In particular in the first lines there are information about the kinematic model of the robot (traction_drive_wheel aka front-tractor tricycle), the kinematic parameters to be estimate, an intial guess of those, the encoder order of the field ticks and their max ranges. These two last information are important to interpret encoder ticks's data.

A record of the dataset is compose by:
	-time: 	       the time stamp
	-ticks:        the reading of the encoders. The first is the steering one which is an absolute encoder, 
			       the second is of the traction and it is a incremental one.
	-model pose:   it is the odometry of our model. An important thing to clarify is that our model does not correspond to the one you have seen in AMR.
				   You are free to formulate the kinematic model as you wish. For your task you can ignore this information.
	-tracker pose: it is the position of the sensor coming from an odometry system for this sensor.
	
Tips on the encoder data: consider that the reading is stored in an uint32 variable, so in some cases it can happen that the variable overflow. Avoid this cases and consider only the incremental information about the encoder to do the integration of your model.

The output of the system has to be:
- 2D position of the sensor w.r.t. the base link
- The kinematic parameters: 
	ksteer: how many radians correspond to one tick
	ktraction: how many meter correspond to one tick
	steer_offset: at which angle correspond the zero of the wheel
	base_line: the lenght of the base_line (remember that the kinematic center is in the middle of the axis of the rear wheels)


