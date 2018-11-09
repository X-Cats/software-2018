package org.usfirst.frc.xcats.robot;


import edu.wpi.first.wpilibj.Joystick;

public class XCatsJSButton {
	Joystick _js;
	int 		_buttonNumber=0;
	int 		_axisNumber=0;
	boolean 	_isAxis = false;
	int 		_direction=1;
	private boolean _toggle = false, _mode = true;	
	

	public XCatsJSButton(Joystick js, int buttonNumber){
		_js = js;
		_buttonNumber = buttonNumber;
		
	}
	public void setState(boolean isTrue){
		_mode = isTrue;
		_toggle = !isTrue;
	}
	public boolean getState()
	{
		return _mode;
	}

	public XCatsJSButton(Joystick js, int axisNumber,boolean isAxis, boolean detectOnPositive){
		_js = js;
		if (isAxis)
			_axisNumber = axisNumber;
		else
			_buttonNumber = axisNumber;
		
		if (detectOnPositive)
			_direction = 1;
		else
			_direction = -1;
		
	}
	
	public boolean isPressed(){
		
		if (_isAxis){
			//mmf 2017-02-13 this has not been tested yet but i think it will work
			double axisValue = _js.getRawAxis(_axisNumber);
			if (axisValue * _direction > 0 && ! _toggle)
				_mode = !_mode;
			
			_toggle = axisValue * _direction > 0;
						
		} else {
			if (_js.getRawButton(_buttonNumber) && ! _toggle)
				_mode = !_mode;
			
			_toggle = _js.getRawButton(_buttonNumber);
		}
		
		return _mode;		
		
	}
}
