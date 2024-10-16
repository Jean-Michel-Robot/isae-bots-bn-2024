# STATE TRANSITIONS

## ACTIVATION [ROS topic /br/idle, message=1]
* IF manager is INACTIVE:
	- Manager switches to state activating --> requests that the motors get ready

## DEACTIVATION [ROS topic /br/idle, message=0]
* IF manager is ACTIVE:
	- Manager switches to state deactivating --> requests that the motors get idle
	
## START TRAJECTORY [ROS topic /nextPositionTeensy]
* IF manager is ACTIVE:
	- Creates a LinearTrajectory between the current estimated position and the target position
	- Switches the controller in state TRAJECTORY, with the linear trajectory, initial linear speed 0 and the preset maximal linear speed and acceleration
	
## START ROTATION [ROS topic /nextPositionTeensy]
* IF manager is ACTIVE:
	- Creates a SetHeadingProfile between the current estimated orientation and the target orientation
	- Switches the controller in state ROTATION, with the profile, initial angular speed 0 and the preset maximal angular speed and acceleration

## BRAKE [ROS topic /nextPositionTeensy]
* IF manager is ACTIVE:
	- If the controller is not already STOPPED or BRAKING:
		* Switches the controller in state BRAKING, with initial (linear and angular) speed=the estimated speed of the robot, and the preset maximal braking deceleration

# UPDATE phase:

## ROS::loop --> 
	* Spins (Spinning is required for ROS to process the received messages; see ROS documentation about "spinning")
	* ControllerManager::loop -->
		* Updates the estimated position (PositionFeedback::update)
		* IF ACTIVATING:
			- Checks if the motors are ready. If they are, switches to state ACTIVE and sets the controller's setpoint to the current estimated position
		* If DEACTIVATING:
			- Checks if the motors are idle. If they are, switches to state INACTIVE
		* IF ACTIVE:
			- Updates the controller (UnicycleController::updateCommand) -->
				* Updates the setpoint (depending on the controller's state, see below)
				* Computes the error between the setpoint and the estimated robot position
				* Computes the command
			- Sends the updated command to the motors
		* Calls the "tick callback" (sends events back to the "haut niveau")
		
	
## SETPOINT UPDATE:

[STATE = STILL/STOPPED] -->
	* Does nothing (does not move the setpoint)
	
[STATE = BRAKING] -->
	* Decreases the target (linear and angular) speed based on the braking deceleration
	* Moves the setpoint accordingly to the target speed
	* If the target speed has reached 0, switches to state STILL/STOPPED
	
[STATE = TRAJECTORY] -->
	* Updates the target linear speed based on the maximal acceleration
		NB: The robot accelerates until it reaches the maximal speed, and starts to slow down when it approaches the end of the trajectory
		If the trajectory is too short, the robot may not reach the maximal speed before it starts to slow down
	* Advances the trajectory by the distance corresponding to the target speed
	* Sets the setpoint to the current position on the trajectory
	* If the trajectory is complete, switches to next state (either STOPPED or ROTATION)

[STATE = ROTATION] -->
	* Same as TRAJECTORY, but with angular speed (and angular distance), and only the orientation of the setpoint changes
	* If the orientation profile is complete, switches to next state (either STOPPED or TRAJECTORY)
	
