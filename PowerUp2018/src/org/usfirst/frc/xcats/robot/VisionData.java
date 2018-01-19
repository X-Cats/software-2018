package org.usfirst.frc.xcats.robot;

public class VisionData
{
    boolean result = false;
    int distance_in_inches = 0;
    int zone = 0;
    double facing_angle_in_deg = 0;
    
    public VisionData()
    {}
    
    public void setResult(boolean res)
    {
    	result = res;
    }
    
    public void setDistanceInInches(int distance)
    {
    	distance_in_inches = distance;
    }
    
    public void setZone(int z)
    {
    	zone = z;
    }
    
    public void setFacingAngleInDeg(double facing_angle)
    {
    	facing_angle_in_deg = facing_angle;
    }
    
    public boolean getResult()
    {
    	return result;
    }
    
    public int getDistanceInInches()
    {
    	return distance_in_inches;
    }
    
    public int getZone()
    {
    	return zone;
    }
    
    public int getFacingAngleInDeg()
    {
    	return (int) facing_angle_in_deg;
    }
}
