package org.usfirst.frc.xcats.robot;


public class Teleop {

	private RobotControls _controls;
	
	public Teleop (RobotControls controls)
	{
		this._controls = controls;
	}
	public void init(){
		_controls.setCoastMode();
	}

	public void execute ()
	{
		try{
			_controls.drive();
			_controls.operate();
			_controls.updateStatus();			
		}
		catch (Exception e){
			System.out.println("* * * DANGER Teleop _controls exception DANGER * * *");
			e.printStackTrace();
		}
	}

}
