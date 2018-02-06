package org.usfirst.frc.xcats.robot;
//package org.usfirst.frc.team191.robot;
//package org.usfirst.frc.team191.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	private RobotControls _controls;
	private ArrayList<AutonomousStep> _steps;

	private int _currentStep=0;
	private float _initialYaw = 0;
	private float _initCompassHeading=0;
	private boolean _angleHasBeenCalculated = false;
	private double _calculatedAngle = 0;
	private VisionData _visionData ;
	private boolean _hasCaptured=false;

	private double _totalAutoTime = Enums.AUTONOMOUS_TIME;
	private AutonomousStep _currentAutoStep;
	private Timer _stepTimer = new Timer();
	private Timer _autoTimer = new Timer();

	private boolean _isExecuting = false;
	private boolean _cancelExecution = false;

	private DigitalOutput _lightsColor = new DigitalOutput(Enums.LIGHTS_ALLIANCE_COLOR);//digial output for controlling color of lights



	private final String _defaultAuto = "Do Nothing";
	private final String _autoForwardOnly = "Go forward only and stop";
	private final String _autoL1 = "L1";
	private final String _autoL2 = "L2";
	private final String _autoCenter = "C";
	private final String _autoR1 = "R1";
	private final String _autoR2 = "R2";
	private final String _autoReadFile = "TextFile read";
	private final String _autoTestSpeed = "Run test time at input speed";
	private final String _autoTestTime = "Test time";
	private final String _autoTestGear = "Test High Gear";
	private final String _autoTestDistance = "Run for 60 in at input speed";
	private final String _autoInTeleop = "TeleopCommands";
	private final String _autoRotator = "Test Rotations";

	//Sendable chooser strings for overide to scale
	private final String _autoCrossCourtYes = "Yes"; //override to score on opposite side of scale
	private final String _autoCrossCourtNo = "No"; //don't override to score on opposite side of scale

	private Navx _navx;

	private String _autoSelected;
	private String _crossCourtSelected;
	private String _gameData;
	private SendableChooser _robotPosition;
	private SendableChooser _crossCourt;


	public Autonomous (RobotControls controls)
	{

		_controls = controls;      	//passes the controls object reference

		_robotPosition = new SendableChooser();
		_robotPosition.addDefault("Default Auto", _defaultAuto);
		_robotPosition.addObject(_autoForwardOnly, _autoForwardOnly);
		_robotPosition.addObject(_autoL1, _autoL1);
		_robotPosition.addObject(_autoL2, _autoL2);
		_robotPosition.addObject(_autoR1, _autoR1);
		_robotPosition.addObject(_autoR2, _autoR2);
		_robotPosition.addObject(_autoCenter, _autoCenter);
		_robotPosition.addObject(_autoReadFile,_autoReadFile);
		_robotPosition.addObject(_autoTestSpeed, _autoTestSpeed);
		_robotPosition.addObject(_autoTestDistance,_autoTestDistance);
		_robotPosition.addObject(_autoRotator, _autoRotator);
		SmartDashboard.putData("Auto choices", _robotPosition);	

		_crossCourt = new SendableChooser();
		_crossCourt.addDefault(_autoCrossCourtNo, _autoCrossCourtNo);
		_crossCourt.addObject(_autoCrossCourtYes, _autoCrossCourtYes);
		SmartDashboard.putData("Cross Court Choices", _crossCourt);	


		/**
		 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
		 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
		 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
		 * below the Gyro
		 *
		 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
		 * If using the SendableChooser make sure to add them to the chooser code above as well.
		 */

		SmartDashboard.putNumber(_autoTestSpeed, 0.5); 			//this is the speed to run the auto calibration test
		//put any properties here on the smart dashboard that you want to adjust from there.
		SmartDashboard.putNumber(_autoTestTime, 3);
		SmartDashboard.putBoolean(_autoTestGear, false);

		System.out.println("auto constructor");

	}

	public Autonomous (RobotControls controls, ArrayList<AutonomousStep> mysteps, double totalTime){
		// this "autonomous" instantiation can be called from teleop to do a set of steps
		_autoSelected = this._autoInTeleop;
		_controls = controls;
		_steps = mysteps;
		_totalAutoTime = totalTime;
		init();	
	}

	public void setSteps(ArrayList<AutonomousStep> mysteps){
		_steps = mysteps;
		init();

	}
	public void init ()
	{
		System.out.println("auto init");

		if (_autoSelected != this._autoInTeleop){
			_autoSelected = (String) _robotPosition.getSelected();
			System.out.println("Auto selected: " + _autoSelected);

			_crossCourtSelected = (String) _crossCourt.getSelected();
			System.out.println("Cross Court Selected: " + _crossCourtSelected);


			//build the steps for the selected autonomous
			setAuto();
		}


		_navx = _controls.getNavx();
		_navx.zeroYaw();
		_initCompassHeading = _navx.getCompassHeading();
		_controls.getDrive().zeroEncoder();
		_initialYaw = _navx.getYaw();
		_currentStep = 0;
		_angleHasBeenCalculated =false;
		_currentAutoStep = null;
		_autoTimer.start();
		_stepTimer.start();
		_isExecuting = false;
		_cancelExecution = false;
		_hasCaptured=false;
		this.updateStatus();


	}

	public void disable(){
		_steps = null;

		updateStatus();

	}
	private void setAuto ()
	{	

		//we are going to construct the steps needed for our autonomous mode
		//		int choice=1;
		double speedTest =SmartDashboard.getNumber(_autoTestSpeed, 0.5);
		int calibrationRunTime = (int) SmartDashboard.getNumber(_autoTestTime, 3);
		boolean highSpeedTest = SmartDashboard.getBoolean(_autoTestGear, false);
		String caseName="";
		_steps =  new ArrayList<AutonomousStep>();

		boolean blueAlliance = false;
		_gameData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putString("GameData", _gameData);

		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
			blueAlliance = true;
			_lightsColor.set(false);//sets lights to match alliance color
		}else
			_lightsColor.set(true);//sets lights to match alliance color

		SmartDashboard.putBoolean("Alliance Color", blueAlliance);
		SmartDashboard.putString("AutoSelected", _autoSelected);
		//			_autoSelected= _auto2;		
		switch (_autoSelected) {
		case _autoL1: 

			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.HIGH_SPEED,"high speed",0,0,0,0));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for shifter",0.1,0,0,0));
			if(_crossCourtSelected == _autoCrossCourtYes && _gameData.charAt(1) == 'R') {
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"First Leg",0,.5,0.5,225));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"Second Leg",0,.4,0.4,207));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"second rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"Third Leg",0,.4,0.4,85));
			}else if(_gameData.charAt(1) == 'L'){
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"First Leg",0,0.5,0.5,285));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"Second Leg",0,.4,0.4,21));
			}else if(_gameData.charAt(0) == 'L'){
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"First Leg",0,.5,0.5,129));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_PROFILE,"Second Leg",0,.4,0.4,34));
			}


			//note we set coastmode in teleop init, but setting it here is a good practice
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.COASTMODE,"Coast Mode",0,0,0,0)); //Set COAST mode for drive train

			break;
		case _autoL2: 
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.7,0.7,112));


			break;

		case _autoCenter: 
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0));
			if (_gameData.charAt(0) == 'L') {
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.7,0.7,45));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.7,0.7,60));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.7,0.7,50));
			}else {
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.7,0.7,45));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.7,0.7,60));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.7,0.7,50));
			}




			break;

		case _autoR1: 
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.HIGH_SPEED,"high speed",0,0,0,0));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for shifter",0.1,0,0,0));
			if(_crossCourtSelected == _autoCrossCourtYes && _gameData.charAt(1) == 'L') {
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.5,0.5,225));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.4,0.4,207));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"second rotation",0,0,0,90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Third Leg",0,.4,0.4,85));
			}else if(_gameData.charAt(1) == 'R'){
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.5,0.5,285));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.4,0.4,21));
			}else if(_gameData.charAt(0) == 'R'){
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.5,0.5,129));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"First rotation",0,0,0,-90));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for rotate",0.1,0,0,0));
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Second Leg",0,.4,0.4,34));
			}


			//note we set coastmode in teleop init, but setting it here is a good practice
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.COASTMODE,"Coast Mode",0,0,0,0)); //Set COAST mode for drive train



			break;

		case _autoR2: 
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"First Leg",0,.7,0.7,112));

			break;



		case _autoRotator:
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.LOW_SPEED,"Low speed transmission",0,0,0,0)); //make sure we are in low speed
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DEADRECON,"Drive Forward1",0,.5,.5,30));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"Turn 60",0,0,0,60));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DEADRECON,"Drive Forward2",0,.5,.5,30));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"Turn 60",0,0,0,-60));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DEADRECON,"Drive Forward3",0,.5,.5,30));

			break;

		case _autoTestSpeed:
			caseName="Speed Test";
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			if(highSpeedTest)
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.HIGH_SPEED,"High Speed",0,0,0,0));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE,"Drive",calibrationRunTime,speedTest,speedTest,0));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.STOP,"Stop",0,0,0,0));
			if(highSpeedTest)
				_steps.add( new AutonomousStep(AutonomousStep.stepTypes.WAIT,"Wait for robot to stop",5,0,0,0));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.COASTMODE,"Coast Mode",0,0,0,0)); //Set COAST mode for drive train
			break;

		case _autoTestDistance:
			caseName="Distance Test";
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.BRAKEMODE,"Brake Mode",0,0,0,0)); //Set brake mode for drive train
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Drive",0,speedTest,speedTest,60));
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.STOP,"Stop",0,0,0,0));
			break;


		default: 
			caseName="Do Nothing";
			_steps.add( new AutonomousStep(AutonomousStep.stepTypes.STOP,"Stop",0,0,0,0));
			break;
		}


		System.out.println("setAuto");
	}

	//	private void addPortcullisSteps(){
	//		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DEADRECON,"Forward to Defense", 0, 0.7, 0.7, (FIRST_LEG_DISTANCE - (Enums.ROBOT_LENGTH_EXTENDED - OVERHANG ) +35)/12.0));
	//		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.LIFT,"Move shifter home", 0.5, 0, 0, 0));			
	//		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DEADRECON,"Navigate Defense", 0, 0.6, 0.6, (DEFENSE_DEPTH - OVERHANG + Enums.ROBOT_LENGTH_EXTENDED)/12.0));
	//	}


	private void addSideRunSteps(boolean isBlueAlliance,boolean isBoilerSide){
		double rotationAngle = 60;
		if (!isBoilerSide){	
			rotationAngle = (isBlueAlliance ? 60 : -60);
		} else {			
			rotationAngle = (isBlueAlliance ? -60 : 60);
		}

		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"backup some more",0,-0.9,-0.9,20)); //back up some more
		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.ROTATE,"Turn to feeder",0,0,0,rotationAngle));
		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE,"start moving",0.5,0.5,0.5,0)); //move some
		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.HIGH_SPEED,"HI speed transmission",0,0,0,0)); //swich tranny
		_steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE,"Drive Down Field",1.0,0.75,0.75,0)); //drive down the field


	}


	public boolean isExecuting(){
		return _isExecuting;
	}
	public void cancelExecution(){
		_cancelExecution = true;
		_controls.setCoastMode();
	}

	public void execute ()
	{
		double encPos = 0; //moved from drivedistance because was initialized twice
		//		System.out.println("auto execute");
		if (_steps == null ){
			_isExecuting = false;
			return;

		}

		if (_steps.size() == 0){
			System.out.println("trying to execute no steps in Autonomous 360");
			_isExecuting = false;	
			return;
		}

		double cTime=0;


		if (_autoTimer.get() > _totalAutoTime || _currentStep >= _steps.size() || _cancelExecution){
			System.out.println("Autonomous Completed: "+ _autoTimer.get());
			_controls.getDrive().set(0, 0, 0, 0);
			_isExecuting = false;	
			disable();
		}
		else
		{
			_currentAutoStep = _steps.get(_currentStep);

			_isExecuting = true;
			//switch (_steps.get(_currentStep))
			switch (_currentAutoStep.stepType)
			{
			case DRIVE:
				drive(_currentAutoStep.stepTime,_currentAutoStep.leftSpeed,_currentAutoStep.rightSpeed);
				break;

			case DRIVE_DISTANCE:
				if (Enums.IS_FINAL_ROBOT)					
					encPos = Math.abs((_currentAutoStep.distance + 4.9437275823) /0.00533638);
				else
					encPos = Math.abs((_currentAutoStep.distance + 4.9437275823) /0.00533638);

				driveStraight(0,_currentAutoStep.leftSpeed,_currentAutoStep.rightSpeed,encPos);

				break;
				
			case DRIVE_PROFILE:
				if (Enums.IS_FINAL_ROBOT)					
					encPos = Math.abs((_currentAutoStep.distance + 4.9437275823) /0.00533638);
				else
					encPos = Math.abs((_currentAutoStep.distance + 4.9437275823) /0.00533638);

				driveStraightProfile(0,_currentAutoStep.leftSpeed,_currentAutoStep.rightSpeed,encPos);

				break;

			case DRIVE_DEADRECON:

				driveStraight(cTime,_currentAutoStep.leftSpeed,_currentAutoStep.rightSpeed,0);
				break;

			case CALCANGLE:
				if (!_angleHasBeenCalculated){

					if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
						//this is where we calculate the rotation angle
						//initial heading + 225 - current heading should be pretty close
						_calculatedAngle = 180 - 30;//_initCompassHeading + 225 - _navx.getCompassHeading();
					}	else{
						_calculatedAngle = 180 + 30; //_initCompassHeading + 135.0 - _navx.getCompassHeading();

					}

					_angleHasBeenCalculated = true;
				}
				SmartDashboard.putNumber("Init Compass Heading", _initCompassHeading);
				SmartDashboard.putNumber("CalcAngle", _calculatedAngle);
				//now just rotate the calculated distance
				rotate(_calculatedAngle);
				break;

			case GET_ANGLE_CORRECTION:
				getVisionCorrection(_currentAutoStep.stepTime);
				break;


			case ROTATE:
				rotate(_currentAutoStep.distance);

				break;


			case BRAKEMODE:
				_controls.setBrakeMode();
				_controls.getDrive().zeroEncoder();
				startNextStep();
				break;

			case COASTMODE:
				_controls.setCoastMode();
				startNextStep();
				break;

			case LOW_SPEED:
				_controls.setLowSpeed();
				startNextStep();
				break;

			case HIGH_SPEED:
				_controls.setHighSpeed();
				startNextStep();
				break;


			case WAIT:
				wait(_currentAutoStep.stepTime);
				break;

			case STOP:
				stop();
				break;


			}
		}

		this.updateStatus();
		_controls.updateStatus();
	}

	private void getVisionCorrection(double time){
		if (!Enums.VISION_CORRECTION_IN_AUTO){
			startNextStep();
			return;
		}

		if (! _hasCaptured){
			_hasCaptured = true;
			if (Enums.CAMERA_USE_REDUCED_BRIGHTNESS)
				_controls.getAutoTarget().setCameraForAuto();

			_visionData = _controls.getAutoTarget().captureImage();	

			if (Enums.CAMERA_USE_REDUCED_BRIGHTNESS)
				_controls.getAutoTarget().setCameraDefaults();			

			if ( _visionData != null)
			{
				if (_visionData.getResult()){
					_controls.getDrive().set(0, 0, 0, 0);
					double correctionOffset = 0.0;

					AutonomousStep nextStep= _steps.get(_currentStep + 1);
					SmartDashboard.putNumber("Facing Angle", _visionData.getFacingAngleInDeg());
					System.out.println("Facing angle: " + _visionData.getFacingAngleInDeg());
					if(Math.abs(_visionData.getFacingAngleInDeg()) < 60 && Math.abs(_visionData.getFacingAngleInDeg()) > 5){

						if (!Enums.AUTO_FROM_CORNER){
							correctionOffset = (_visionData.getFacingAngleInDeg() > 0 ? 2.0 : -2.0);							
						}
						nextStep.distance = _visionData.getFacingAngleInDeg() - correctionOffset; // we seem to be off about 2 deg				
					}
					else{
						System.out.println("Angle found but not in range for correction!");
						startNextStep();
					}
				}
				startNextStep();
			}
		}
		else 
			startNextStep();


	}

	private void rotate( double distance){
		//float deltaYaw;
		double  speed =0.25;
		double tolerance=0.5;
		int direction=1;

		
		if (distance == 0){
			startNextStep();
			return;
		}

		//deltaYaw = _initialYaw + _controls.getNavx().getYaw();
		//SmartDashboard.putNumber("deltaYaw", deltaYaw);
		// 
		direction = (distance > 0 ? -1 : 1);
		speed = direction * speed;	
		_controls.getDrive().set(speed, speed, -speed, -speed);


		if(Math.abs(_controls.getNavx().getYaw()) > Math.abs(distance)){
			SmartDashboard.putNumber("Auto Yaw", _controls.getNavx().getYaw());
			speed=-speed/1.5;
			_controls.getDrive().set(speed, speed, -speed, -speed);
			if(Math.abs(_controls.getNavx().getYaw())-Math.abs(distance)<=tolerance){
				_controls.getDrive().set(0, 0, 0, 0);
				startNextStep();
			}
		}
		//System.out.println("Rotating: "+ distance + " speed "+speed);
		
	}




	public void updateStatus(){

		if (_steps != null && _currentAutoStep != null){
			SmartDashboard.putNumber("Step Count", _steps.size());
			SmartDashboard.putString("Current Command", this._currentStep + " " + _currentAutoStep.name  + "\n " + _currentAutoStep.stepTime);			
		}

	}
	public void drive (double time, double left, double right)
	{


		if (_stepTimer.get() > time)
		{
			_controls.getDrive().set(0, 0, 0, 0);
			SmartDashboard.putNumber("Encoder Value", _controls.getDrive().getAbsAvgEncoderValue());
			startNextStep();
		}
		else
		{
			_controls.getDrive().set(-left, -left, -right, -right);
		}
	}

	public void driveStraight (double time, double left, double right,double targetEncPosition)
	{
		float deltaYaw;

		//deltaYaw = _initialYaw - _controls.getNavx().getYaw();
		deltaYaw = _navx.getYaw();
		double offsetLimit = 0.05;
		double offset=0;

		SmartDashboard.putNumber("currentYaw", _initialYaw);
		SmartDashboard.putNumber("deltaYaw", deltaYaw);
		if (left == right){
			offset = Math.abs(deltaYaw);
			if(offset > offsetLimit){
				offset = offsetLimit;
			}
			if(deltaYaw < 0){
				left = left * (1+offset);
				right = right * (1-offset);
			}else{
				left = left * (1-offset);
				right = right * (1+offset);
			}
		}

		SmartDashboard.putNumber("Auto Enc", _controls.getDrive().getAbsAvgEncoderValue());
		if (targetEncPosition > 0){
			if (_controls.getDrive().getAbsAvgEncoderValue() >= targetEncPosition)
			{
				_controls.getDrive().set(0, 0, 0, 0);
				System.out.println("Target encoder position" + targetEncPosition);
				System.out.println("Current encoder position" + _controls.getDrive().getAbsAvgEncoderValue());
				startNextStep();
			}
			else
			{
				_controls.getDrive().set(-left, -left, -right, -right);
			}

		} else {
			if (_stepTimer.get() >= time)
			{
				_controls.getDrive().set(0, 0, 0, 0);
				System.out.println("Target encoder position" + targetEncPosition);
				System.out.println("Current encoder position" + _controls.getDrive().getAbsAvgEncoderValue());
				startNextStep();
			}
			else
			{
				_controls.getDrive().set(-left, -left, -right, -right);
			}
		}
	}

	public void driveStraightProfile (double time, double left, double right,double targetEncPosition)
	{
		double remainingDistance = 0;
		double remainingPercent = 0;
		remainingDistance = targetEncPosition - _controls.getDrive().getAbsAvgEncoderValue();
		remainingPercent = remainingDistance/targetEncPosition;
		double leftsign = (left >= 0) ? 1:-1;
		double rightsign = (right >= 0) ? 1:-1;
		 
		
	if (remainingDistance <= 2500) {
		left = 0.30 * leftsign;
		right = 0.30 * rightsign;
	}else if (remainingDistance <= 4000){
		left = 0.35 * leftsign;
		right = 0.35 * rightsign;
	}else if (remainingDistance <= 5000){
		left = 0.40 * leftsign;
		right = 0.40 * rightsign;
	}else if (remainingDistance <= 6000){
		left = 0.45 * leftsign;
		right = 0.45 * rightsign;
	}else if (remainingDistance <= 7000){
		left = 0.50 * leftsign;
		right = 0.50 * rightsign;
	}	
		
		float deltaYaw;

		//deltaYaw = _initialYaw - _controls.getNavx().getYaw();
		deltaYaw = _navx.getYaw();
		double offsetLimit = 0.05;
		double offset=0;

		SmartDashboard.putNumber("currentYaw", _initialYaw);
		SmartDashboard.putNumber("deltaYaw", deltaYaw);
		if (left == right){
			offset = Math.abs(deltaYaw);
			if(offset > offsetLimit){
				offset = offsetLimit;
			}
			if(deltaYaw < 0){
				left = left * (1+offset);
				right = right * (1-offset);
			}else{
				left = left * (1-offset);
				right = right * (1+offset);
			}
		}

		SmartDashboard.putNumber("Auto Enc", _controls.getDrive().getAbsAvgEncoderValue());
		if (targetEncPosition > 0){
			if (_controls.getDrive().getAbsAvgEncoderValue() >= targetEncPosition)
			{
				_controls.getDrive().set(0, 0, 0, 0);
				System.out.println("Target encoder position" + targetEncPosition);
				System.out.println("Current encoder position" + _controls.getDrive().getAbsAvgEncoderValue());
				startNextStep();
			}
			else
			{
				_controls.getDrive().set(-left, -left, -right, -right);
			}

		} else {
			if (_stepTimer.get() >= time)
			{
				_controls.getDrive().set(0, 0, 0, 0);
				System.out.println("Target encoder position" + targetEncPosition);
				System.out.println("Current encoder position" + _controls.getDrive().getAbsAvgEncoderValue());
				startNextStep();
			}
			else
			{
				_controls.getDrive().set(-left, -left, -right, -right);
			}
		}
	}
	public void drive (double time, double leftX, double leftY, double rightX, double rightY)
	{
		if (_stepTimer.get() > time)
		{
			_controls.getDrive().set(0, 0, 0, 0);
			startNextStep();
		}
		else
		{
			_controls.getDrive().set(leftX, leftY, rightX, rightY);
		}
	}

	public void grab (int grabSetpoint)
	{
		//			_controls.getElevator().setGrabMotor(grabSetpoint);
		startNextStep();
	}








	public void wait (double time)
	{
		if (_stepTimer.get() > time)
			startNextStep();
	}



	public void stop ()
	{
		_controls.getDrive().set(0, 0, 0, 0);
		startNextStep();
	}

	public void startNextStep ()
	{
		System.out.println("Step "+_currentStep + " "+ _stepTimer.get() + " s  --"+   _currentAutoStep.name  + "-- is completed. EncPos= "+_controls.getDrive().getAbsAvgEncoderValue());
		_navx.zeroYaw();
		SmartDashboard.putNumber("Starting Yaw", _controls.getNavx().getYaw() );
		_currentStep++;
		_stepTimer.reset();
		_angleHasBeenCalculated=false;
		_controls.getDrive().zeroEncoder();		
	}	

	private void ReadAutoFile(){
		BufferedReader br = null;

		try {

			String sCurrentLine;
			String temp[];	
			String sComment = "\\\\";
			String sSteps = "";
			AutonomousStep newStep;


			//			br = new BufferedReader(new FileReader("C:\\autonomous\\testing.txt"));
			br = new BufferedReader(new FileReader("/home/lvuser/autonomous.txt"));

			while ((sCurrentLine = br.readLine()) != null) {				
				if (sCurrentLine.startsWith(sComment)){
					System.out.println("Comment: ingoring -> "+ sCurrentLine);
				} else{
					System.out.println(sCurrentLine);
					sSteps = sCurrentLine + "\n" + sSteps; 
					temp = sCurrentLine.split(",");
					newStep =  new AutonomousStep();
					newStep.name = temp[1];
					newStep.stepType = AutonomousStep.stepTypes.valueOf( temp[0]);
					newStep.stepTime = Double.parseDouble(temp[2]);
					newStep.leftSpeed = Double.parseDouble(temp[3]);
					newStep.rightSpeed = Double.parseDouble(temp[4]);
					newStep.distance = Double.parseDouble(temp[5]);
					_steps.add(newStep);

					System.out.println("Autosteps now has "+_steps.size());
				}
			}
			SmartDashboard.putString("File Steps", sSteps);

		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if (br != null)br.close();
			} catch (IOException ex) {
				ex.printStackTrace();
			}
		}

	}			

}
