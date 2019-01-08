package org.usfirst.frc.xcats.robot;

public class Climber {

    private XCatsSpeedController _climber;

    public Climber(){
        _climber = new XCatsSpeedController("Climber", Enums.CLIMBER_CAN_ID, true, XCatsSpeedController.SCType.TALON, null, null);
    }

    //run climber forward
    public void climb(){
    	_climber.set(Enums.CLIMBER_SPEED);

    }

    public void stop(){
    	_climber.set(0);

    }

    public double getClimbSpeed()
    {
        return _climber.get();
    }

}
