package org.usfirst.frc.xcats.robot;


public class Enums {

	/*
	public static final int FRONT_LEFT_DRIVE = 0,
			REAR_LEFT_DRIVE = 1,
			FRONT_RIGHT_DRIVE = 2,
			REAR_RIGHT_DRIVE = 3;
	public static final int CAN_FRONT_LEFT_DRIVE = 1,
			CAN_REAR_LEFT_DRIVE = 2,
			CAN_FRONT_RIGHT_DRIVE = 3,
			CAN_REAR_RIGHT_DRIVE = 4;
			*/
	
	 //front left, rear left, front right, rear right
	public static final int PDP_CAN_ID = 20;
	public static final int PCM_CAN_ID = 21;
	public static final double SPEED_REDUCTION_FACTOR = 1.0;
	
	/*
	 * offset to calibrate so that at 0.5 on the right side, the left side is going at the same travel
	 * right = right + speed_calibration * right;
	 * so if the right side is going 5% farther than the left when the speeds are the same
	 * the right = right + -0.05 * right
	*/
	public static final double SPEED_CALIBRATION = 0.0;  
	
	public static final boolean IS_FINAL_ROBOT = true;
	public static final String  DRIVE_CONTROLLER_TYPE = "Talon"; // choices are "Jaguar" or "Talon"
	public static final boolean HAS_MECHANUM_WHEELS = false;
	public static final boolean USE_PID = false;    	// This is if the drive train is using PID control
	public static final int     MAX_CAN_SPEED = 6000;
	public static final boolean USE_CAN = true;			// if the motors are wired with CAN bus use this
	public static final double  MOTOR_STOP_TIME = .1;	//when using coast mode this is how to stop gradually	
	public static final boolean USE_NAVX = true;		//when using NAVX set this to true;
	public static final boolean USE_COMPRESSOR = true;  //set to true when using a compressor
	public static final boolean USE_SOFTWARE_SPEED_REDUCTION = false; 	//set to true only if you wish to use the trigger button to engage a sofware reduction of speed low/high
	public static final boolean USE_2SC_TANK = true;     //when true, then the robot drive is 2 motor controllers and the rest are followers
	public static final double BROWNOUT_VOLTAGE_THRESHOLD = 7.5;
	public static final double BROWNOUT_VOLTAGE_REDUCTIONFACTOR = 0.5;
	
	public static final double  ROBOT_LENGTH_COMPACT = 30.0; //length of robot with shooter in home position
	public static final double  ROBOT_LENGTH_EXTENDED = 40; // length of robot with shooter down

	//vision constants
	public static final boolean VISION_SYSTEM_ENABLED = true;
	public static final int CAMERA_FOV_HORIZONTAL = 61;  // Horizontal field of view for MSoft Lifecam
	public static final boolean CAMERA_USE_REDUCED_BRIGHTNESS = false; //reduces brightness during acquisition for vision processing
//	public static final int PIXEL_PER_DEGREE = 14;   // commented out to allow for various resolutions
//	public static final int CAMERA_X_PIXELS_TOTAL = 640;  // commented out to allow for various resolutions
//	public static final int CAMERA_Y_PIXELS_TOTAL = 480;  // commented out to allow for various resolutions
	public static final double PEG_LENGTH = 10.5;  // inches
	public static final double PEG_CHANNEL_DEPTH = 3.375; // inches
	public static final double CAMERA_DIST_FROM_FRONT = 5.0;  // inches
	public static final boolean CAMERA_SAVE_IMAGES = true; //
	public static final boolean VISION_CORRECTION_IN_AUTO = true;

	//these values keep track specifically of the specific motor controllers
	//if we only need 2 motors in the drive, use FRONT_LEFT and FRONT_RIGHT. Make sure that the arrays below have a length of 2
	public static final int  REAR_LEFT = 0,  FRONT_LEFT = 1, AUX_LEFT = 2, REAR_RIGHT = 3, FRONT_RIGHT = 4,  AUX_RIGHT = 5 ;
//	public static final int DRIVE_MOTOR_NUMBERS[] = {FRONT_LEFT, FRONT_RIGHT}; //if we do not use CAN bus, the motors are created in this sequence
	public static final int     DRIVE_MOTOR_NUMBERS[] = { REAR_LEFT, FRONT_LEFT, AUX_LEFT,  REAR_RIGHT, FRONT_RIGHT, AUX_RIGHT}; //if we do not use CAN bus, the motors are created in this sequence

	//before we put the encoders on drive because the cable was not long enough to reach the "Front" motor
	//public static final int     CAN_DRIVE_MOTOR_NUMBERS[] = {1, 2, 3, 4, 5, 6}; //these are the CAN bus ids of the motors
	public static final int     CAN_DRIVE_MOTOR_NUMBERS[] = {1, 3, 2, 4, 6, 5}; //these are the CAN bus ids of the motors
		
	//This is for the use of the compressor
	public static final int     PCM_SHIFTER_FORWARD = 4, PCM_SHIFTER_REVERSE=5;  //this is used to shift the gear ration on the drive train from low to high (SHIFTER)
	public static final double  SHIFTER_DELAY_TIME = 0.25;  // this time is used for the "slack speed" inbetween gear shifts
	public static final double  SHIFTER_DELAY_SPEED = 0.4;  // this is the speed for the "slack" during shifting
		
	//Joysticks
	public static final int LEFT_DRIVE_JS = 1, RIGHT_DRIVE_JS = 2, DRIVE_JS = 1, OPERATOR_JS = 0;
	public static final boolean TWO_JOYSTICKS = true;
	public static final boolean DASHBOARD_INPUT = false, DASHBOARD_OUTPUT = false;
	
	//Following enums need to be set for 2018 robot, current values are just placeholders
	
	//Acquisition
	public static final int LEFT_ACQUISITION_CAN_ID = 30;//can id for left acquisition motor
	public static final int RIGHT_ACQUISITION_CAN_ID = 31;//can id for right acquisition motor
	public static final double ACQUISITION_SPEED = 0.5;//speed of acquisition motors, is currently test value
	
	//Elevator
	public static final int ELEVATOR_MASTER_CAN_ID = 32;//can id for elevatorMaster
	public static final int ELEVATOR_FOLLOWER_CAN_ID = 32;//can id for elevatorFollower
	public static final double ELEVATOR_SPEED = 0.5;//speed for elevator motor, currently test value
	
	//Autonomous uses the chooser object to select mode
	public static final int AUTONOMOUS_TIME = 15;
	public static final int AUTO_SWITCH_ANALOG = 0;
	public static final boolean AUTO_FROM_CORNER = false;
	
	//Lights
	public static final int GEAR_LIGHTS_CHANNEL = 5;//output for controlling the lights while ejecting the gear
	public static final int WINCH_LIGHTS_CHANNEL = 4;//output for controlling lights while climbing
	public static final int LIGHTS_ALLIANCE_COLOR = 3;//output for controlling lights to match alliance color
	
}
