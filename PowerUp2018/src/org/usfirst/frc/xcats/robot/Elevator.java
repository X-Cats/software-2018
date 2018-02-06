package org.usfirst.frc.xcats.robot;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;


public class Elevator {
	
	private XCatsSpeedController _elevatorMaster;
	private XCatsSpeedController _elevatorFollower;
	
	public Elevator() {
		_elevatorMaster = new XCatsSpeedController("Elevator", Enums.ELEVATOR_MASTER_CAN_ID, true, SCType.TALON, null, null);
		//_elevatorFollower = new XCatsSpeedController("Elevator", Enums.ELEVATOR_FOLLOWER_CAN_ID, true, SCType.TALON, null, null);
		//_elevatorFollower.setFollower(Enums.ELEVATOR_MASTER_CAN_ID);
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

}
