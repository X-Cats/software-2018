package org.usfirst.frc.xcats.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Acquisition {

	private XCatsSpeedController _leftAcquisition;
	private XCatsSpeedController _rightAcquisition;
	private DoubleSolenoid _armsSolenoid;
	private Relay _linkage1;
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

	public Acquisition() {
		_leftAcquisition = new XCatsSpeedController("Left Acquisition", Enums.LEFT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition = new XCatsSpeedController("Right Acquisition", Enums.RIGHT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		//		_rightAcquisition.setInverted(true);
		//		_rightAcquisition.setFollower(Enums.LEFT_ACQUISITION_CAN_ID);


		_armsSolenoid = new DoubleSolenoid(Enums.PCM_CAN_ID, Enums.PCM_ARMS_IN, Enums.PCM_ARMS_OUT);
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);

		this._linkage1 = new Relay(Enums.LINKAGE_ONE_CHANNEL, Relay.Direction.kBoth);
	}

	//intake cube
	public void intake() {
		_leftAcquisition.set(-Enums.ACQUISITION_SPEED);
		_rightAcquisition.set(Enums.ACQUISITION_SPEED);
	}

	//push out cube
	public void release() {
		_leftAcquisition.set(Enums.ACQUISITION_SPEED);
		_rightAcquisition.set(-Enums.ACQUISITION_SPEED);
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
		this._linkage1.set(Relay.Value.kReverse);
	}

	public void lowerLinkage() {

		this._linkage1.set(Relay.Value.kForward);
	}

	public void autoLowerLinkage() {
		if(!this._linqMovingDown) {
			this._linqUp.stop();
			this._linqUp.reset();
			this._linqUp.start();
			this.raiseLinkage();
			this._linqMovingDown = true;
		}
	}

	public void autoRaiseLinkage(){
		if(!this._linqMovingUp) {
			this._linqDown.stop();
			this._linqDown.reset();
			this._linqDown.start();
			this.lowerLinkage();
			this._linqMovingUp = true;
		}
	}
	
	public void stopLinkage() {
		this._linkage1.set(Relay.Value.kOn);
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
			}else{
				this.raiseLinkage();
			}
		}

		if(_linqMovingDown){
			if(_linqDown.get() >= Enums.LINQ_DOWN_TIMER){
				this.stopLinkage();
			}else
				this.lowerLinkage();
		}
		lk = this._linkage1.get().name();



//		SmartDashboard.putString("Linkage Relay Value", lk);
//
//		SmartDashboard.putString("Arms Solenoid Value", this._armsSolenoid.get().name());
			
		SmartDashboard.putBoolean("Arms Open", this._armsOpen);

	}
}
