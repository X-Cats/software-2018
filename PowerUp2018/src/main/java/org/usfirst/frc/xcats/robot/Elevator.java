package org.usfirst.frc.xcats.robot;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator {

	private XCatsSpeedController _elevatorMaster;
	private XCatsSpeedController _elevatorFollower;
	private DigitalInput _switchLimit;
	private DigitalInput _scaleLimit;
	private DigitalInput _bottom;
	private DoubleSolenoid _climberSolenoid;

	private int _setPoint;
	private int _targetEncoder;
	private boolean _elevatorMoving;
	private DigitalInput _targetLimit;
	private boolean _moveToHome = false;
	private double _elevatorDownSpeed = Enums.ELEVATOR_SPEED_DOWN;

	public Elevator() {
		_climberSolenoid = new DoubleSolenoid(Enums.PCM_CAN_ID, Enums.PCM_CLIMB_IN, Enums.PCM_CLIMB_OUT);
		_climberSolenoid.set(DoubleSolenoid.Value.kReverse);


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
		_targetLimit = null;
	}

	public void init() {
		this._elevatorDownSpeed = Enums.ELEVATOR_SPEED_DOWN;
		this._climberSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void raise(double setPoint) {
		double speed = setPoint * Enums.ELEVATOR_SPEED_UP;
		this.terminateMotion();
		if(this.isAtBottom() && speed < 0) {
			this.stop();
			return;
		}
		if(this.isAtScale() && speed > 0) {
			this.stop();
			return;
		}
		_elevatorMaster.set(speed);
		//System.out.println("Raise");
	}

	public void lower(double setPoint) {
		double speed = setPoint * _elevatorDownSpeed * -1;
		if(this.scaleEncoder()< Enums.ELEVATOR_SWITCH_SET_POINT) {
			speed = 0.5 * setPoint;
		}
		this.terminateMotion();
		if(this.isAtBottom() && speed < 0) {
			this.stop();
			return;
		}
		if(this.isAtScale() && speed > 0) {
			this.stop();
			return;
		}
		_elevatorMaster.set(speed);
		//System.out.println("Lower");
	}

	public void stop() {
		this.terminateMotion();
		_elevatorMaster.set(0.075);


	} 
	//	//used in auto to bring elevator down if it is not
	//	public void bringElevatorDown() {
	//		if(!this.isAtBottom()) {
	//			this._elevatorMaster.set( _elevatorDownSpeed);
	//		}



	public void goToBottom() {

		int deltaEncoder;

		System.out.println("Check Bottom: "+ this.isAtBottom()+" "  + this.scaleEncoder() +" " +_targetEncoder );

		_setPoint = Enums.ELEVATOR_BOTTOM_SET_POINT;
		if(!this.isAtBottom() &&  Math.abs(this.scaleEncoder() - this._setPoint ) > Enums.ELEVATOR_ENCODER_SAFETY ) {
			System.out.println("GOTO BOTTOM");	
			_elevatorMoving = true;
			_targetLimit = this._bottom;		

			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				//System.out.println("Going down");
				double speed = _elevatorDownSpeed;
				if(this.scaleEncoder()< Enums.ELEVATOR_SWITCH_SET_POINT) {
					speed = 0.5 * _elevatorDownSpeed;
				}
				this._elevatorMaster.set( speed);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				//System.out.println("Going up");
				this._elevatorMaster.set( Enums.ELEVATOR_SPEED_UP);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
			}
		}		


	}

	public void goToSwitch() {
		int deltaEncoder;

		//System.out.println("Check Switch: "+ this.isAtSwitch()+" "  + this.scaleEncoder() +" " +_targetEncoder );

		_setPoint = Enums.ELEVATOR_SWITCH_SET_POINT;
		if(!this.isAtSwitch() &&  Math.abs(this.scaleEncoder() - this._setPoint ) > Enums.ELEVATOR_ENCODER_SAFETY ) {
			//System.out.println("GOTO SWITCH");	
			_elevatorMoving = true;
			_targetLimit = this._switchLimit;		

			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				//	System.out.println("Going down");
				this._elevatorMaster.set( _elevatorDownSpeed);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				//System.out.println("Going up");
				this._elevatorMaster.set( Enums.ELEVATOR_SPEED_UP);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
			}
		}
	}

	public void goToScale() {

		int deltaEncoder;

		//System.out.println("Check Scale: "+ this.isAtScale()+" "  + this.scaleEncoder() +" " +_targetEncoder );

		_setPoint = Enums.ELEVATOR_SCALE_SET_POINT;
		if(!this.isAtScale() &&  Math.abs(this.scaleEncoder() - this._setPoint ) > Enums.ELEVATOR_ENCODER_SAFETY ) {
			//System.out.println("GOTO SCALE");	
			_elevatorMoving = true;
			_targetLimit = this._scaleLimit;		

			deltaEncoder = (int) (this.scaleEncoder() - this._setPoint);
			if(deltaEncoder > 0) {
				//	System.out.println("Going down");
				this._elevatorMaster.set( _elevatorDownSpeed);
				_targetEncoder = this._setPoint - Enums.ELEVATOR_ENCODER_SAFETY;
			}else if(deltaEncoder < 0) {
				//System.out.println("Going up");
				this._elevatorMaster.set( Enums.ELEVATOR_SPEED_UP);
				_targetEncoder = this._setPoint  + Enums.ELEVATOR_ENCODER_SAFETY;
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

	public double heightPercent() {
		return this.scaleEncoder()/Enums.ELEVATOR_SCALE_SET_POINT;
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
	public boolean isAtTarget() {
		return !_targetLimit.get();
	}

	public double scaleEncoder() {
			return -this._elevatorMaster.getEncPosition();
	}
	
	public DigitalInput getTargetLimit() {
		return this._targetLimit;
	}



	public boolean getElevatorMoving() {
		return this._elevatorMoving;
	}

	public double getTargetEncoder() {
		return this._targetEncoder;
	}

	public void prepareForClimb() {
		//if(DriverStation.getInstance().getMatchTime() <= Enums.ENGAME_TIME) {
		_elevatorDownSpeed = Enums.ELEVATOR_SPEED_ENDGAME;
		_climberSolenoid.set(DoubleSolenoid.Value.kForward);

		//}
	}



	public void updateStatus() {

		int deltaEncoder;

//		SmartDashboard.putBoolean("Bottom Limit", isAtBottom());
//		SmartDashboard.putBoolean("Switch Limit", isAtSwitch());
//		SmartDashboard.putBoolean("Scale Limit", isAtScale());

		//SmartDashboard.putNumber("Elevator Encoder Value", this.scaleEncoder());
		//SmartDashboard.putNumber("Elevator Target Encoder Value", this._targetEncoder);

		if(isAtBottom()) {
			this.zeroEncoder();
		}
		
		

		double elevatorPercent = this.heightPercent();
//		SmartDashboard.putNumber("Elevator Percent", elevatorPercent);
		
//		SmartDashboard.putNumber("Elevator Encoder Position", this._elevatorMaster.getEncPosition());

		// move elevator to setpoint
		if(this._targetLimit != null) {
			if(this.isAtTarget() || Math.abs(this.scaleEncoder() - _targetEncoder) <= Enums.ELEVATOR_ENCODER_SAFETY) {
				//	System.out.println("Stop Reached Limit: "+ this.isAtTarget() +" "  + this.scaleEncoder() +" " +_targetEncoder );
				this._elevatorMaster.set(0);
				this.terminateMotion();
				_targetLimit = null;
				_elevatorMoving = false;
			}
		}


	}

}
