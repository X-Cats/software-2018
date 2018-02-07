package org.usfirst.frc.xcats.robot;

public class Climber {

    private XCatsSpeedController _climber;

    public Climber(){
        _climber = new XCatsSpeedController("Climber", Enums.CLIMBER_CAN_ID, true, XCatsSpeedController.SCType.TALON, null, null);
    }

    public void climb(){

    }

    public void stop(){

    }

}
