package org.usfirst.frc.xcats.robot;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

public class Acquisition {
	
	private XCatsSpeedController _leftAcquisition;
	private XCatsSpeedController _rightAcquisition;
	
	public Acquisition() {
		_leftAcquisition = new XCatsSpeedController("Left Acquisition", Enums.LEFT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition = new XCatsSpeedController("Right Acquisition", Enums.RIGHT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition.setFollower(Enums.LEFT_ACQUISITION_CAN_ID);
	}
	
	public void intake() {
		_leftAcquisition.set(Enums.ACQUISITION_SPEED);
	}
	
	public void release() {
		_leftAcquisition.set(-Enums.ACQUISITION_SPEED);
	}
	
	public void stop() {
		_leftAcquisition.set(0);
	}

}
