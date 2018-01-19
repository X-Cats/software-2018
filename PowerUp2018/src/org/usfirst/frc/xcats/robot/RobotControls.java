package org.usfirst.frc.xcats.robot;


import java.util.ArrayList;

import com.ctre.CANTalon;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * Buttons and inputs:
 * 
 * Driver's joysticks
 *		button		Action
 *		6			Shift between low and high speed. The compressor is used to do this, but the DO controls the signal to the solenoid that shifts the gears
 * 
 * 
 * Operator's joysticks
 *		button		Action
 *		1			reset the NAVX status 
 * 
 * 
 * 
 * 
 */


public class RobotControls {
	private Joystick _leftJS, _rightJS, _driveJS, _operatorJS;
	
	private XCatsDrive _drive;
	private Timer _shiftTimer;
	private boolean _shifting=false;

	private boolean _slowMode = true;
	private boolean _liftMode = false;
	private XCatsJSButton _speedToggleButton;
	private XCatsJSButton _highSpeedButton;

	private boolean _highSpeed = false;
	private DoubleSolenoid _dblSolShifter;
	private Navx _navx;
	private boolean _driveStraight=false;
	private float _initialYaw=0;
	
	private VisionData _visionData;
	
	private Compressor _compressor;
	private PowerDistributionPanel _pdp;
		
	private Autonomous _commandAuto;
	private AutoTarget _autoTarget;
	private boolean _autoMode=false; 

	public RobotControls ()
	{

		//
		_shiftTimer = new Timer();
		_pdp = new PowerDistributionPanel(Enums.PDP_CAN_ID);
		
		//simple XCatsDrive, no PID etc
		_drive = new XCatsDrive (Enums.USE_CAN,true);
		_drive.setDashboardIO(false, false);
		_drive.setInverted();
		_drive.setCoastMode();
		_drive.setMagneticEncoders(false);
		_drive.zeroEncoder();
		
		_autoTarget = new AutoTarget(false);
		
		//_autoTarget.setCameraForAuto();

		//_drive.setPDP(_pdp, Enums.BROWNOUT_VOLTAGE_THRESHOLD, Enums.BROWNOUT_VOLTAGE_REDUCTIONFACTOR);
		
		
		if (Enums.USE_COMPRESSOR){
			_compressor = new Compressor(Enums.PCM_CAN_ID);
			_compressor.start();
			if(Enums.IS_FINAL_ROBOT){
				
			}else{
			_dblSolShifter = new DoubleSolenoid(Enums.PCM_CAN_ID,Enums.PCM_SHIFTER_FORWARD,Enums.PCM_SHIFTER_REVERSE);
			}
		}
	
	    //the NAVX board is our gyro subsystem	
		if (Enums.USE_NAVX){
			_navx= new Navx(this);
			_navx.resetStatus();
			_navx.zeroYaw();			
		}
		

		
		if (Enums.TWO_JOYSTICKS)
		{
			_leftJS = new Joystick(Enums.LEFT_DRIVE_JS);
			_rightJS = new Joystick(Enums.RIGHT_DRIVE_JS);
			_speedToggleButton = new XCatsJSButton(_rightJS,1);
		}
		else{
			
			_driveJS = new Joystick(Enums.DRIVE_JS);
			_speedToggleButton = new XCatsJSButton(_driveJS,6);
		}

		_operatorJS = new Joystick(Enums.OPERATOR_JS);
				
		try
		{
//			CameraServer camera = CameraServer.getInstance();
//			camera.startAutomaticCapture(0);
		}
		catch (Exception e)
		{
			System.out.println(e);
			e.printStackTrace();
		}
		
		// always initialize the robot in low gear
		_dblSolShifter.set(DoubleSolenoid.Value.kReverse);
		_slowMode = (_dblSolShifter.get() == DoubleSolenoid.Value.kReverse ? true : false);
		System.out.println("State of shifter ="+ _dblSolShifter.get());
		System.out.println("Slow mode" +_slowMode);
//		setLowSpeed();
		
	}
	
	public void setCoastMode(){
		_drive.setCoastMode();
	}
	public void setBrakeMode(){
		_drive.setBrakeMode();
	}
	public Navx getNavx(){
		return _navx;
	}
	public void setHighSpeed(){
		
		if (!_slowMode)
			return;
		else
			shiftTransmission();		
	}
	public void setLowSpeed(){
		
		if (_slowMode)
			return;
		else
			shiftTransmission();
		
	}

	private void executeCommandAuto(){
		//check to see if we have some commands
		if (_commandAuto == null)
			return;
		
		//if we are done clear the memory
		if (_commandAuto.isExecuting()){
			System.out.println(". . . Executing commands for vision system response!");
			_commandAuto.execute();
		}else {
			System.out.print("......Auto Completed for vision system response!");
			_commandAuto = null;
			_autoMode = false;
		}
		
	}
	private void prepAuto(){

		if (_commandAuto != null)
			return;
		
		if (_visionData != null){
			SmartDashboard.putBoolean("Vision Processing", _visionData.result);
			SmartDashboard.putNumber("Vision FACING Angle", _visionData.facing_angle_in_deg);
			SmartDashboard.putNumber("Vision ZONE", _visionData.zone);
			SmartDashboard.putNumber("Vision DISTANCE", _visionData.distance_in_inches);			
		}
		
		
		System.out.println("Prepping commands for vision system response!");
		
		
		_autoMode = true;
		ArrayList<AutonomousStep> steps;		
		steps =  new ArrayList<AutonomousStep>();
		
		steps.add( new AutonomousStep(AutonomousStep.stepTypes.DRIVE_DISTANCE,"Drive Forward",0,.5,.5,12.0)); //assuming 99.64 inches
		steps.add( new AutonomousStep(AutonomousStep.stepTypes.GEAR,"Place Gear",0,0,0,60));
		steps.add( new AutonomousStep(AutonomousStep.stepTypes.STOP,"Stop",0,0,0,0));	
		
		_commandAuto = new Autonomous(this,steps,5);
		_commandAuto.execute();
	}
	
	private void shiftTransmission(){

		_slowMode = !_slowMode;
		if (_speedToggleButton.getState() != _slowMode){
			_speedToggleButton.setState(_slowMode);}

		_shiftTimer.reset();
		_shiftTimer.start();
		_shifting = true;
		
//		if (_dblSolShifter.get() == DoubleSolenoid.Value.kOff)
//			_slowMode = true;
		
		if (_slowMode)
			_dblSolShifter.set(DoubleSolenoid.Value.kReverse);
		 else 
			_dblSolShifter.set(DoubleSolenoid.Value.kForward);										
		
		_drive.zeroEncoder();
		
	}
	public void drive ()
	{

//		SmartDashboard.putBoolean("isEjecting", _gear.isEjecting());
//		SmartDashboard.putBoolean("isShifting", _shifting);
//		
		
		//if we are executing commands then execute it but exit response to driver.
		//we will need to allow some escape sequence to cancel this, maybe respond to both triggers on the drivers joysticks
		if (_commandAuto != null){
			SmartDashboard.putBoolean("executing command", true);
			this.executeCommandAuto();
			return;
		} else {
			SmartDashboard.putBoolean("executing command", false);
		}
			
		boolean reductionToggle = _slowMode;
//		int directionLeft = 1; // this is used to tell if we are going forward or backwards. The shifting speed needs to be in the same direction!
//		int directionRight = 1;

		if (!_shifting){
			if (Enums.TWO_JOYSTICKS){
				if (_rightJS.getRawButton(11)){
					if (_driveStraight){
						driveStraight(0.4,0.4);
						
					}else{
						_navx.zeroYaw();
						_initialYaw = _navx.getYaw();
						_driveStraight = true;
					}
					
				} else{
					_driveStraight = false;
					_drive.set(_leftJS, _rightJS);
				}
			}
			else {
				_drive.set(_driveJS);
			}
		}

		
		//only transition on the "downstroke" of the button, we dont care how long it is held
		reductionToggle  = _speedToggleButton.isPressed();
		if (reductionToggle != _slowMode){
			shiftTransmission();							
		}

//		if (_navx != null){
//			
//			if (Enums.TWO_JOYSTICKS){
//				boolean aToggle = _leftJS.getRawButton(3);
//				if (aToggle != _autoMode ){
//					System.out.println("PREP auto");
//					this.prepAuto();
//				}
//			}
//			else {
//				if(_driveJS.getRawButton(5)){
//					this.prepAuto();
//				}
//			}
//		}
		
		//image capture
		if (Enums.VISION_SYSTEM_ENABLED){
			if(Enums.TWO_JOYSTICKS){
				//button 1 is the "trigger" button
				if (_leftJS.getRawButton(1)){
					
					if (Enums.CAMERA_USE_REDUCED_BRIGHTNESS)
						_autoTarget.setCameraForAuto();
					
					_visionData = _autoTarget.captureImage();
					SmartDashboard.putBoolean("Vision Processing", _visionData.getResult());
					SmartDashboard.putNumber("Vision FACING Angle", _visionData.getFacingAngleInDeg());
					SmartDashboard.putNumber("Vision ZONE", _visionData.getZone());
					SmartDashboard.putNumber("Vision DISTANCE", _visionData.getDistanceInInches());
					
					if (Enums.CAMERA_USE_REDUCED_BRIGHTNESS)
						_autoTarget.setCameraDefaults();
									
				}
			}else{
				if(_driveJS.getRawButton(7)){
					_visionData = _autoTarget.captureImage();
				}
			}
		}

	}

	public void driveStraight ( double left, double right)
	{
		float deltaYaw;

		deltaYaw =  _navx.getYaw();
		double offset=0;

//		SmartDashboard.putNumber("currentYaw", _initialYaw);
//		SmartDashboard.putNumber("deltaYaw", deltaYaw);
		double  offsetLimit = 0.05;
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
//			System.out.println("Drive Straight " + deltaYaw +"   "+ offset +"      " + left +"    "+ right);
			_drive.set(-left, -left, -right, -right);
	}
	
	
	public void operate ()
	{
		//if we are executing commands then exit response to operator
		if (_commandAuto != null){
			return;
		}
		
		
		//these feeder buttons are mutually exclusive, but need to be on a 
		//different thumb from the feeder raise and lower
		
	
		
		
	}



	public XCatsDrive getDrive()
	{
		return _drive;
	}
	public AutoTarget getAutoTarget(){
		return _autoTarget;
	}

	
	public void updateStatus ()
	{
		try {
//			SmartDashboard.putNumber("pdp total current", _pdp.getTotalCurrent());
//			SmartDashboard.putNumber("pdp total energy",_pdp.getTotalEnergy());
//			SmartDashboard.putNumber("pdp total power" ,_pdp.getTotalPower());
//			SmartDashboard.putNumber("pdp temperature",_pdp.getTemperature());
//			SmartDashboard.putNumber("pdp voltage",_pdp.getVoltage());			
		}
		catch (Exception e){
			System.out.println("error reading PDP... RobotControls.updateStatus");
		}
	
		//we need to drop the speed for a small time frame so that the gears can handle the gear shift
		//so this loop will detect the shifting command and at the end of it will resume speed
		//get the direction of the drive
		if (_shifting ){
			int directionLeft = 1; // this is used to tell if we are going forward or backwards. The shifting speed needs to be in the same direction!
			int directionRight = 1;
			
			if (_drive.get(Enums.FRONT_LEFT)< 0)
				directionLeft = -1;
			
			if (_drive.get(Enums.FRONT_RIGHT)< 0)
				directionRight = -1;

			if (_shiftTimer.get() >= Enums.SHIFTER_DELAY_TIME)
				_shifting = false;				
			else if (Math.abs(_drive.get(Enums.FRONT_LEFT)) > Enums.SHIFTER_DELAY_SPEED)
				_drive.set( directionLeft * Enums.SHIFTER_DELAY_SPEED,  directionRight * Enums.SHIFTER_DELAY_SPEED);			
		}
		
//		SmartDashboard.putNumber("LeftSpeed", _drive.get(Enums.FRONT_LEFT));
//		SmartDashboard.putNumber("Direction", directionLeft);
//		SmartDashboard.putBoolean("Shifter in HIGH Gear", !_slowMode);		
//		
//		SmartDashboard.putBoolean("DriverLeftButton", _leftJS.getRawButton(3));
		
		_drive.updateStatus();
		_autoTarget.updateStatus();
		
		if (_commandAuto != null)
			_commandAuto.updateStatus();
		
		if (_navx != null){
			_navx.updateStatus();
			
		}
		
	}
}