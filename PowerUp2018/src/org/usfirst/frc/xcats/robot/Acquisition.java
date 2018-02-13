package org.usfirst.frc.xcats.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.xcats.robot.XCatsSpeedController.SCType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Acquisition {
	
	private XCatsSpeedController _leftAcquisition;
	private XCatsSpeedController _rightAcquisition;
	private DoubleSolenoid _armsSolenoid;
	private Relay _linkage1;
	private Relay _linkage2;
	
	public Acquisition() {
		_leftAcquisition = new XCatsSpeedController("Left Acquisition", Enums.LEFT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition = new XCatsSpeedController("Right Acquisition", Enums.RIGHT_ACQUISITION_CAN_ID, true, SCType.TALON, null, null);
		_rightAcquisition.setFollower(Enums.LEFT_ACQUISITION_CAN_ID);
		_rightAcquisition.setInverted(true);
		
		_armsSolenoid = new DoubleSolenoid(Enums.PCM_CAN_ID, Enums.PCM_ARMS_IN, Enums.PCM_ARMS_OUT);
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);

		this._linkage1 = new Relay(Enums.LINKAGE_ONE_CHANNEL, Relay.Direction.kBoth);
		this._linkage2 = new Relay(Enums.LINKAGE_TWO_CHANNEL, Relay.Direction.kBoth);
	}
	
	//intake cube
	public void intake() {
		_leftAcquisition.set(Enums.ACQUISITION_SPEED);
	}
	
	//push out cube
	public void release() {
		_leftAcquisition.set(-Enums.ACQUISITION_SPEED);
	}
	
	public void stop() {
		_leftAcquisition.set(0);
	}
	
	//push secondary wheels for acquisition in
	public void armsIn() {
		_armsSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	//pull secondary wheels for acquisition out
	public void armsOut() {
		_armsSolenoid.set(DoubleSolenoid.Value.kReverse);	
	}
	
	public void raiseLinkage() {
		this._linkage1.set(Relay.Value.kForward);
		this._linkage2.set(Relay.Value.kForward);
	}
	
	public void lowerLinkage() {
		this._linkage1.set(Relay.Value.kReverse);
		this._linkage2.set(Relay.Value.kReverse);
	}
	
	public void stopLinkage() {
		this._linkage1.set(Relay.Value.kOff);
		this._linkage2.set(Relay.Value.kOff);
	}

	public void updateStatus() {
	  String lk = "";
		
//		switch (this._linkage1.get())
//		{
//		case kForward :
//			lk = "Forward";
//			break;
//		case kReverse :
//			lk = "Reverse";
//			break;
//		case kOff:
//			lk = "Off";
//			break;
//		case kOn:
//			lk = "On";
//			break;
//		}
		
		lk = this._linkage1.get().name();
		
		String k = this._linkage2.get().name();
		System.out.println("Linkage 2 Relay Value" + k);
		
		SmartDashboard.putString("Linkage 2 Relay Value", k);
		
		
		SmartDashboard.putString("Linkage Relay Value", lk);
		
	}
}
