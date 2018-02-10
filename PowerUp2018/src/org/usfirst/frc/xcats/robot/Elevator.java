package org.usfirst.frc.xcats.robot;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator {
	
	private XCatsSpeedController _elevatorMaster;
	private XCatsSpeedController _elevatorFollower;
	private DigitalInput _switchLimit;
	private DigitalInput _scaleLimit;
	private DigitalInput _bottom;
	private DigitalInput _top;
	
	public Elevator() {
		_bottom = new DigitalInput(Enums.ELEVATOR_BOTTOM_LIMIT);
		_switchLimit = new DigitalInput(Enums.ELEVATOR_SWITCH_LIMIT);
		_scaleLimit = new DigitalInput(Enums.ELEVATOR_SCALE_LIMIT);
		_top = new DigitalInput(Enums.ELEVATOR_TOP_LIMIT);
		
		_elevatorMaster = new XCatsSpeedController("Elevator Master", Enums.ELEVATOR_MASTER_CAN_ID, true, SCType.TALON, _bottom, _top);
		_elevatorFollower = new XCatsSpeedController("Elevator Follower", Enums.ELEVATOR_FOLLOWER_CAN_ID, true, SCType.TALON, null, null);
		_elevatorFollower.setFollower(Enums.ELEVATOR_MASTER_CAN_ID);
	}
	
	public void raise() {
		_elevatorMaster.set(Enums.ELEVATOR_SPEED);
	}
	
	public void lower() {
		_elevatorMaster.set(-Enums.ELEVATOR_SPEED);
	}
	
	public void stop() {
		_elevatorMaster.set(0);
	}
	
	public void goToSwitch() {
		
	}
	
	public void goToScale() {
		
	}
	
	public void raiseLinkage() {
		
	}
	
	public void lowerLinkage() {
		
	}
	
	public void stopLinkage() {
		
	}

}
