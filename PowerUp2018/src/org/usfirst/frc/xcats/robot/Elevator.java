package org.usfirst.frc.xcats.robot;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator {

	private XCatsSpeedController _elevatorMaster;
	private XCatsSpeedController _elevatorFollower;
	private DigitalInput _switchLimit;
	private DigitalInput _scaleLimit;
	private DigitalInput _bottom;

	private int _setPoint;
	private int _targetEncoder;
	private boolean _elevatorMoving;
	private DigitalInput _targetLimit;
	private boolean _moveToHome = false;

	public Elevator() {
		//these need to be changed to reflect their actual positions
		_bottom = new DigitalInput(Enums.ELEVATOR_BOTTOM_LIMIT);
		_switchLimit = new DigitalInput(Enums.ELEVATOR_SWITCH_LIMIT);
		_scaleLimit = new DigitalInput(Enums.ELEVATOR_SCALE_LIMIT);

		_elevatorMaster = new XCatsSpeedController("Elevator Master", Enums.ELEVATOR_MASTER_CAN_ID, true, SCType.TALON, _bottom, null);
		_elevatorFollower = new XCatsSpeedController("Elevator Follower", Enums.ELEVATOR_FOLLOWER_CAN_ID, true, SCType.TALON, null, null);
		_elevatorFollower.setFollower(Enums.ELEVATOR_MASTER_CAN_ID);
		this._elevatorMaster.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.zeroEncoder();

		_setPoint = 0;
		
		_elevatorMoving = false;
		_targetLimit = _switchLimit;
	}

	public void raise(double setPoint) {
		this.terminateMotion();
		_elevatorMaster.set(setPoint * Enums.ELEVATOR_SPEED_UP * -1);
	}

	public void lower(double setPoint) {
		this.terminateMotion();
		_elevatorMaster.set(setPoint * Enums.ELEVATOR_SPEED_DOWN);
	}

	public void stop() {
		_elevatorMaster.set(0);
	}
	
	public void goToBottom() {
		int deltaEncoder;
		
		_setPoint = Enums.ELEVATOR_BOTTOM_SET_POINT;
		_targetLimit = this._bottom;
		if(!_elevatorMoving) {
			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				this._elevatorMaster.set(Enums.ELEVATOR_SPEED_DOWN);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				this._elevatorMaster.set(Enums.ELEVATOR_SPEED_UP);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
			}else {
				_targetEncoder = (int) this.scaleEncoder();
			}
		}

		
	}

	public void goToSwitch() {
		int deltaEncoder;

		_setPoint = Enums.ELEVATOR_SWITCH_SET_POINT;
		_targetLimit = this._switchLimit;
		if(!_elevatorMoving) {
			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				this._elevatorMaster.set(-0.5);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				this._elevatorMaster.set(0.7);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
			}else {
				_targetEncoder = (int) this.scaleEncoder();
			}
		}
	}

	public void goToScale() {
		int deltaEncoder;
		_setPoint = Enums.ELEVATOR_SCALE_SET_POINT;
		_targetLimit = this._scaleLimit;
		
		if(!_elevatorMoving) {
			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				this._elevatorMaster.set(-0.5);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				this._elevatorMaster.set(0.7);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
			}else {
				_targetEncoder = (int) this.scaleEncoder();
			}
		}
	}
	
	public void terminateMotion() {
		this._targetLimit = null;
		this._elevatorMoving = false;
	}

	public void zeroEncoder() {
		_elevatorMaster.zeroEncoder();
	}

	
	public boolean isAtBottom () {
		return !_bottom.get();
	}
	public boolean isAtSwitch() {
		return ! _switchLimit.get();
	}
	
	public boolean isAtScale() {
		return !_scaleLimit.get();
	}
	
	public double scaleEncoder() {
		return -this._elevatorMaster.getEncPosition();
	}
	
	public boolean getElevatorMoving() {
		return this._elevatorMoving;
	}
	
	
	public void updateStatus() {

		int deltaEncoder;

		SmartDashboard.putBoolean("Bottom Limit", isAtBottom());
		SmartDashboard.putBoolean("Switch Limit", isAtSwitch());
		SmartDashboard.putBoolean("Scale Limit", isAtScale());

		SmartDashboard.putNumber("Elevator Encoder Value", this.scaleEncoder());
		SmartDashboard.putNumber("Elevator Target Encoder Value", this._targetEncoder);

		if(isAtBottom())
			this.zeroEncoder();
		

		
        //move elevator to setpoint
		if(this._targetLimit != null) {
		if(!_targetLimit.get() || this._elevatorMaster.getEncPosition() == _targetEncoder) {
			System.out.println("Triggered Stop Condition");
			this._elevatorMaster.set(0);
			this.terminateMotion();
		}
		}


	}

}
