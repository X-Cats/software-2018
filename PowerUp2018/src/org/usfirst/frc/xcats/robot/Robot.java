package org.usfirst.frc.xcats.robot;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

//package org.usfirst.frc.team191.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Thread visionThread;
    
    Teleop _teleop;
    Autonomous _auto;
    RobotControls _controls;
    AutoTarget _autoTarget;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        
    	try{
    		visionThread = new Thread(() -> {
    			_autoTarget = new AutoTarget();
    			
    			while (!Thread.interrupted()) {
    				_autoTarget.processImage();
    			}
    		});
    		visionThread.setDaemon(true);
    		visionThread.start();    			

            _controls = new RobotControls();
            _teleop = new Teleop(_controls);
            _auto = new Autonomous(_controls);    		
    		
    		
    	}
    	catch (Exception e)
    	{
    		System.out.println("* * * * CANNOT INIT ROBOT * * * *");
    		e.printStackTrace();
    	}
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	try {
        	_auto.init();    		
    	}
    	catch (Exception e){
    		//keep going!
    		e.printStackTrace();
    	}
    }
    
    public void teleopInit() {
    	try {
        	_auto.disable();    
        	_teleop.init();
    	}
    	catch (Exception e){
    		//keep going!
    		e.printStackTrace();
    	}
    }


    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	try {
    		
        	_auto.execute();    		
    	}
    	catch (Exception e)
    	{
    		e.printStackTrace();
    	}
    }
    
    public void disabledPeriodic(){
      	try {
        	_auto.disable();    		
    	}
    	catch (Exception e)
    	{
    		// keep going! ignore the error
    	}  
    	
    }
    public void disabledInit(){
    	
    }
    public void robotPeriodic(){
    	
    }
  
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	try {
    		_teleop.execute();    		
    	}
    	catch (Exception e)
    	{
    		// keep going! ignore the error
    	}  
    }
    
 
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
