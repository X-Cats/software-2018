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
	private Relay _linkage2;
	private boolean _movingHome = false;
	private Timer _acqTimer = new Timer();
	private Timer _linqTimer = new Timer();
	private Timer _cubeOut = new Timer();
	private Boolean _cubeEjecting = false;
	
	public Acquisition() {
		_leftAcquisition = new XCatsSpeedController("Left Acquisition", Enums.LEFT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition = new XCatsSpeedController("Right Acquisition", Enums.RIGHT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
//		_rightAcquisition.setInverted(true);
//		_rightAcquisition.setFollower(Enums.LEFT_ACQUISITION_CAN_ID);

		
		_armsSolenoid = new DoubleSolenoid(Enums.PCM_CAN_ID, Enums.PCM_ARMS_IN, Enums.PCM_ARMS_OUT);
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);

		this._linkage1 = new Relay(Enums.LINKAGE_ONE_CHANNEL, Relay.Direction.kBoth);
		this._linkage2 = new Relay(Enums.LINKAGE_TWO_CHANNEL, Relay.Direction.kBoth);
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
	
	
	public void stop() {
		_leftAcquisition.set(0);
		_rightAcquisition.set(0);
	}
	
	//push secondary wheels for acquisition in
	public void armsIn() {
		System.out.println("arms in");
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	//pull secondary wheels for acquisition out
	public void armsOut() {
		System.out.println("arms out");
		_armsSolenoid.set(DoubleSolenoid.Value.kReverse);	
	}
	
	public void raiseLinkage() {
		this._linkage1.set(Relay.Value.kReverse);
		this._linkage2.set(Relay.Value.kReverse);
	}
	
	public void lowerLinkage() {
		
		this._linkage1.set(Relay.Value.kForward);
		this._linkage2.set(Relay.Value.kForward);
	}
	
	public void stopLinkage() {
		this._linkage1.set(Relay.Value.kOn);
		this._linkage2.set(Relay.Value.kOn);
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
		  if (_linqTimer.get() >= Enums.LINQ_HOME_TIME) {
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
		
		lk = this._linkage1.get().name();
		
		String k = this._linkage2.get().name();
		
		SmartDashboard.putString("Linkage 2 Relay Value", k);
		
		
		SmartDashboard.putString("Linkage Relay Value", lk);

		SmartDashboard.putString("Arms Solenoid Value", this._armsSolenoid.get().name());
		
	}
}
