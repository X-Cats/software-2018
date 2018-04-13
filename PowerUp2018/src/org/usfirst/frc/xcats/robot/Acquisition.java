package org.usfirst.frc.xcats.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Acquisition {

	private XCatsSpeedController _leftAcquisition;
	private XCatsSpeedController _rightAcquisition;
	private DoubleSolenoid _armsSolenoid;
	private XCatsSpeedController _linkage;
	private boolean _movingHome = false;
	private Timer _acqTimer = new Timer();
	private Timer _linqTimer = new Timer();
	private Timer _cubeOut = new Timer();
	private Timer _cubeIn = new Timer();
	private Timer _linqUp = new Timer();
	private Timer _linqDown = new Timer();
	private Boolean _cubeEjecting = false;
	private Boolean _cubeIntake = false;
	private boolean _armsOpen = false;
	private boolean _linqMovingUp = false;
	private boolean _linqMovingDown = false;
	private DigitalInput _linkageBottom;
	private AnalogPotentiometer _linkagePot;

	public Acquisition() {
		_leftAcquisition = new XCatsSpeedController("Left Acquisition", Enums.LEFT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition = new XCatsSpeedController("Right Acquisition", Enums.RIGHT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		//		_rightAcquisition.setInverted(true);
		//		_rightAcquisition.setFollower(Enums.LEFT_ACQUISITION_CAN_ID);


		_armsSolenoid = new DoubleSolenoid(Enums.PCM_CAN_ID, Enums.PCM_ARMS_IN, Enums.PCM_ARMS_OUT);
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);

		this._linkage = new XCatsSpeedController("Four Bar Linkage", Enums.LINKAGE_CAN_ID,true,SCType.TALON,null,null);

		this._linkageBottom = new DigitalInput(Enums.LINKAGE_BOTTOM_LIMIT);
		
		this._linkagePot = new AnalogPotentiometer(Enums.LINKAGE_POT_CHANNEL);
	}

	//intake cube
	public void intake() {
		_leftAcquisition.set(-Enums.ACQUISITION_SPEED_IN);
		_rightAcquisition.set(Enums.ACQUISITION_SPEED_IN);
	}

	//push out cube
	public void release() {
		_leftAcquisition.set(Enums.ACQUISITION_SPEED_OUT);
		_rightAcquisition.set(-Enums.ACQUISITION_SPEED_OUT);
	}

	public void cubeOut() {
		if (!_cubeEjecting) {
			this._cubeOut.reset();
			this._cubeOut.start();

			this.release();

			this._cubeEjecting = true;
		}
	}

	public void cubeIn() {
		if (!_cubeIntake) {
			this._cubeIn.reset();
			this._cubeIn.start();

			this.intake();

			this._cubeIntake = true;
		}
	}


	public void stop() {
		_leftAcquisition.set(0);
		_rightAcquisition.set(0);
	}

	//push secondary wheels for acquisition in
	public void armsIn() {
		System.out.println("arms in");
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);
		this._armsOpen = false;
	}

	//pull secondary wheels for acquisition out
	public void armsOut() {
		System.out.println("arms out");
		_armsSolenoid.set(DoubleSolenoid.Value.kReverse);
		this._armsOpen = true;
	}

	public void toggleArms() {
		if(this._armsSolenoid.get() == DoubleSolenoid.Value.kReverse) {
			this._armsSolenoid.set(DoubleSolenoid.Value.kForward);
			this._armsOpen = false;
		}else {
			this._armsSolenoid.set(DoubleSolenoid.Value.kReverse);
			this._armsOpen = true;
		}
	}

	public void raiseLinkage() {
		if(this._linkagePot.get() <= Enums.LINKAGE_UP_LIMIT)
			this.stopLinkage();
		else
			this._linkage.set(Enums.LINKAGE_SPEED_UP);
	}

	public void lowerLinkage() {
		//if(Enums.IS_FINAL_ROBOT) {
			if(this._linkagePot.get() >= Enums.LINKAGE_DOWN_LIMIT)
				this.stopLinkage();
			else
				this._linkage.set(Enums.LINKAGE_SPEED_DOWN);
//		}else
//			this._linkage.set(Enums.LINKAGE_SPEED_DOWN);
	}

	public void autoLowerLinkage() {
		if(!this._linqMovingDown) {
			this.lowerLinkage();
			this._linqMovingDown = true;
		}
	}

	public void autoRaiseLinkage(){
		if(!this._linqMovingUp) {
			this._linqUp.stop();
			this._linqUp.reset();
			this._linqUp.start();
			this.raiseLinkage();
			this._linqMovingUp = true;
		}
	}

	public void stopLinkage() {
		this._linkage.set(0);
	}

	public void moveToHome() {
		if (!_movingHome) {

			this.armsIn();
			_linqTimer.reset();
			_linqTimer.start();


			_movingHome = true;
		}

	}
	public void updateStatus() {
		String lk = "";

		if (_movingHome) {
			if (_linqTimer.get() >= Enums.LINQ_UP_TIMER) {
				_linqTimer.stop();
				this.stopLinkage();
				_movingHome = false;

			}else {
				this.raiseLinkage();
			}
		}

		if(_cubeEjecting) {
			if (_cubeOut.get() >= Enums.RELEASE_TIMER) {
				this._cubeOut.stop();
				this.stop();
				_cubeEjecting = false;

			}else {
				this.release();
			}
		}

//		SmartDashboard.putBoolean("Linkage Moving Up", this._linqMovingUp);
//		SmartDashboard.putBoolean("Linkage Moving Down", this._linqMovingDown);


		if(_cubeIntake) {
			if (_cubeIn.get() >= Enums.INTAKE_TIMER) {
				this._cubeIn.stop();
				this.stop();
				_cubeIntake = false;

			}else {
				this.intake();
			}
		}

		if(_linqMovingUp) {
			if (_linqUp.get() >= Enums.LINQ_UP_TIMER) {
				this.stopLinkage();
				this._linqMovingUp = false;
			}else{
				this.raiseLinkage();
			}
		}

		if(_linqMovingDown){
			if(this._linkageBottom.get()){
				this.stopLinkage();
				this._linqMovingDown = false;
			}else
				this.lowerLinkage();
		}

//		SmartDashboard.putBoolean("Four Bar Down", this._linkageBottom.get());



		//		SmartDashboard.putString("Linkage Relay Value", lk);
		//
		//		SmartDashboard.putString("Arms Solenoid Value", this._armsSolenoid.get().name());

		SmartDashboard.putBoolean("Arms Open", this._armsOpen);
		
		SmartDashboard.putNumber("Linkage Pot Value", _linkagePot.get());

	}
}
