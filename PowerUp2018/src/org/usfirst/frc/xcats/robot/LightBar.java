package org.usfirst.frc.xcats.robot;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;

public class LightBar {
/*	private class PWM {
		private PWM (int i) {
			System.out.println("  Fake PWM instance created");			
		}
		protected void setSpeed (double pat) {
//			System.out.println("  Fake PWM setSpeed called");
		}
	}
	private class Timer {
		private double secs;
		private Timer () {
			secs = 10;
			System.out.println("  Fake Timer instance created");			
		}
		protected void reset () {
			secs = 10;
		}
		protected double get () {
			return --secs;
		}
	}*/
	private PWM controller = new PWM(Enums.LED_LIGHT_BAR);
	// constants for 5V LED operation
	private static double NO_PATTERN = 0; // I think this is true.  Test it.
	private static double SolidDarkRed = 0.59f;
	private static double SolidRed = 0.61f;
	private static double SolidYellow = 0.69f;
	private static double SolidGreen = 0.77f;
	private static double BlueViolet = 0.89f; // Use for FeedMeACube signal
	private static double SolidBlack = 0.99f;
	private static double COLOR_WAVES = -0.43f; // Color Waves, Party Palette
	private static double COLLISION = -0.07f; // Fixed Palette Pattern; Strobe, Gold

	// Set true if you want to see light events on console, else false.
	private static boolean debugToConsole = false;
	private long t0 = System.currentTimeMillis();
	private long t1 = t0;
	
	// Can use a counter or timer for event-driven light flash.
	// At 50 Hz, counter assumes being called every 20 usec.
	private static int MS_PER_CYCLE = 20;
	private int countdown; // count of 50 = 1 sec
	
	private Timer lightTimer = new Timer();
	private boolean timerRunning = false;
	// #seconds at the start of the timer, so you can determine when it's done
	private double timerStartCount; 
	private double endtime = 0; 
	
	private boolean isTimerOverridable;
	private double defaultPattern = COLOR_WAVES;
	private double currentPattern = NO_PATTERN;
	private double previousPattern = NO_PATTERN;
	
	public LightBar() {
		setPattern(defaultPattern);
	}

	public void setToRed() {
		debugPrint("*** Go Red ***");
		setPattern(SolidRed);
	}

	public void setToRed(int timeval) {
		debugPrint("*** Go Red for a spell ***");
		controller.setSpeed(SolidRed);
		endtime = timeval;
		if (lightTimer.get() > 1){
			debugPrint("Light timer changed after: "+ lightTimer.get() + " seconds");
		}
	}

	public void setCollisionEvent() {
		debugPrint("***Collision detected***");
		setEventPattern(COLLISION, 1000); // show for 1 sec
	}

	public void setGiveMeCubeEvent() {
		debugPrint("***Tell Human to feed Cube to Robot***");
		setEventPattern(SolidYellow, 1000); // show for 1 sec
	}
	
	/* 
	 * Pseudo-code for timed light events.
	 * If currentPattern != 0 AND !timerRunning
	 *   then: 
	 *     Set previousPattern = currentPattern.
	 *     Set currentPattern = new event pattern.
	 *     Set timerRunning to countdownValue
	 */

	public void setToYellow() {
		debugPrint("*** Go Yellow ***");
		setPattern(SolidYellow);
	}

	public void setToGreen() {
		debugPrint("*** Go Green ***");
		setPattern(SolidGreen);
	}

	public void setToBlueViolet() {
		debugPrint("*** Go BlueViolet ***");
		setPattern(BlueViolet);
	}

	public void lightTimerCheck() {
		debugPrint("*** Go Green, from lightTimerCheck() ***");
		controller.setSpeed(SolidGreen);
	}

	/* This should be called as part of a robots general execution loop, 
	 * in case any timed light has expired.  This is to be during 
	 * both autonomous and teleop periods.
	 */
	public void execute() {
		execute2();
	}

	// To be called as part of the general execution loop, in case any timed light has expired
	// If countdown timer is used.
	private void execute2() {
		if ((countdown > 0) && (--countdown == 0)) { // then see if timed event is over
			debugPrint("<LIGHTS> Timed light event is over.");
			if (previousPattern != NO_PATTERN) {
				controller.setSpeed(previousPattern);
				debugPrint("<LIGHTS> Returning to previous pattern: " + previousPattern);
				previousPattern = NO_PATTERN;
			} else { 
				controller.setSpeed(defaultPattern);
				debugPrint("<LIGHTS> Returning to default pattern: " + defaultPattern);
			}
		}
	}

	// To be called as part of the general execution loop, in case any timed light has expired
	// If WPI Timer object is used.
	private void execute1() {
		if (timerRunning) {
			if (lightTimer.get() >= endtime) {
				debugPrint("<LIGHTS> WPI light timer completed after: "+ lightTimer.get() + " seconds");
				lightTimer.reset();
			}
		}
		controller.setSpeed(SolidYellow);
	}
	
	private void setPattern(double pat) {
		if (countdown > 0) {
			previousPattern = pat;
			debugPrint("<LIGHTS> Set NEXT pattern to: " + pat);
		} else {
			controller.setSpeed(pat);	
			debugPrint("<LIGHTS> Set pattern to: " + pat);		
		}
		currentPattern = pat;
	}
	
	/* 
	 * An event (timed) light pattern will supersede a previous event pattern.
	 * It also saves and returns to the previous pattern when the timer is done
	 */
	private void setEventPattern(double pat, int ms) {
		if (countdown == 0) { // i.e. if not currently in a timed event already
			previousPattern = currentPattern;
			currentPattern = pat;
		} 
		controller.setSpeed(pat);
		countdown = ms / MS_PER_CYCLE;
		debugPrint("<LIGHTS> Set timed pattern to: " + pat);
	}
	

//	private void debugPrint(String s) {
	public void debugPrint(String s) {
		if (debugToConsole) {
			t1 = System.currentTimeMillis();
			//int secs = (t1-t0)/1000;
			int deciSecs = (int) ((t1-t0)/100);
			int secs = deciSecs / 10;
			deciSecs -= (secs*10);
//			String lapsedTime = (t1-t0)/1000 + "." + (t1-t0)/100;
			String lapsedTime = secs + "." + deciSecs;
			System.out.print(lapsedTime + " -- ");
			System.out.println(s);
		}
	}


	/*
	 * Pattern#  PulseWidthInuSec SPARKvalue  
	 * 47 1465 -0.07 Fixed Palette Pattern; Strobe, Gold
	 * 80 1795 0.59 Solid Colors Dark red
	 * 81 1805 0.61 Solid Colors Red
	 * 82 1815 0.63 Solid Colors Red Orange
	 * 83 1825 0.65 Solid Colors Orange
	 * 84 1835 0.67 Solid Colors Gold
	 * 85 1845 0.69 Solid Colors Yellow
	 * 86 1855 0.71 Solid Colors Lawn Green
	 * 87 1865 0.73 Solid Colors Lime
	 * 88 1875 0.75 Solid Colors Dark Green
	 * 89 1885 0.77 Solid Colors Green
	 * 90 1895 0.79 Solid Colors Blue Green
	 * 91 1905 0.81 Solid Colors Aqua
	 * 92 1915 0.83 Solid Colors Sky Blue
	 * 93 1925 0.85 Solid Colors Dark Blue
	 * 94 1935 0.87 Solid Colors Blue
	 * 95 1945 0.89 Solid Colors Blue Violet
	 * 96 1955 0.91 Solid Colors Violet
	 * 97 1965 0.93 Solid Colors White
	 * 98 1975 0.95 Solid Colors Gray
	 * 99 1985 0.97 Solid Colors Dark Gray
	 * 100 1995 0.99 Solid Colors Black
	 */


	/**
	 * Developer test harness shows the splash screen for a fixed length of 
	 * time, without launching the full application.
	 */
	private static void main(String... aArgs){
//	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String testName = "LightBarTest";
		System.out.println("Executing "+testName+" ...");
		LightBar lb = new LightBar();
		int cycleCounter = 0;
		
		lb.setToBlueViolet();
		cycleCounter = 250; // 5 seconds
		while (cycleCounter-- != 0) {
			try {
				Thread.sleep(MS_PER_CYCLE); // pause for 20 ms seconds (1 50 HZ cycle)
				lb.execute();				
			}
			catch(InterruptedException ex) {
				System.out.println(ex);
			}			
		}
		lb.setToRed();
		
		cycleCounter = 500; // 10 seconds
		while (cycleCounter-- != 0) {
			try {
				Thread.sleep(MS_PER_CYCLE); // pause for 20 ms seconds (1 50 HZ cycle)
				lb.execute();
				if (cycleCounter == 400) { // after 2 sec
					lb.setCollisionEvent();
				}
				if (cycleCounter == 375) { // after another 0.5 sec
					lb.setGiveMeCubeEvent();
				}
			}
			catch(InterruptedException ex) {
				System.out.println(ex);
			}			
		}
		
		cycleCounter = 500; // 10 seconds
		lb.debugPrint("Timer test: 10 ...");
		while (cycleCounter-- != 0) {
			try {
				Thread.sleep(MS_PER_CYCLE); // pause for 20 ms seconds (1 50 HZ cycle)
				lb.execute();
			}
			catch(InterruptedException ex) {
				System.out.println(ex);
			}			
		}
		lb.debugPrint("Testing over.");
		System.out.println("... Done with "+testName+" ...");
		System.exit(0);
	}
}

