package org.usfirst.frc.xcats.robot;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class GearPlacementVision
{
	public void GearPlacementVision()
	{

	}

	public VisionData processImage(Mat _mat)
	{
		boolean result = false;
		int distance_in_inches = 0;
		int zone = 0;
		double facing_angle_in_deg = 0;
		int res_x = 640;  // default value of image resolution, horizontally (e.g. 640x480) 
		int res_y = 320;  // default
		res_x = _mat.width();
		res_y = _mat.height();
		
		double pixels_per_degree_X = res_x / Enums.CAMERA_FOV_HORIZONTAL;  // Horizontal field of view for MSoft Lifecam
		System.out.println("Horizontal resolution: " + res_x + 
				"  Horizontal pixels per degree = " + pixels_per_degree_X);
		VisionData visionData = new VisionData();

		System.loadLibrary( Core.NATIVE_LIBRARY_NAME);
		long t0 = System.currentTimeMillis();
		System.out.println("Extracting distance/angle from processed image...\n");

		GripPipeline gp = new GripPipeline();
		gp.process(_mat);   

		ArrayList<Rect> rectList = new ArrayList<Rect>();
		int rectNum = 0;

		for (MatOfPoint mop : gp.filterContoursOutput())
		{
			Rect rect = Imgproc.boundingRect(mop);
			if ((rect.x > 5) && (rect.y > 0.25*res_y) && (rect.y < 0.75*res_y )) {
				rectList.add(rect);
				System.out.println("Rectangle #" + rectNum
						+ ": Height=" + rect.height
						+ ", Width=" + rect.width
						+ ", Area=" + rect.area()
						+ ", X=" + rect.x
						+ ", Y=" + rect.y);
				rectNum++;
			}
		}

		if (rectList.size() == 0)
		{
			System.out.println("VISION: NOT ACCURATE - OPERATOR CONTROL NEEDED!!!  Found 0 reflections!");
			visionData.setResult(false);
			return visionData;
		}

		Rect left = null;
		Rect right = null;
		Rect taller = null;

		if ( (rectList.size() < 2) )
		{
			System.out.println("VISION: NOT ACCURATE - OPERATOR CONTROL NEEDED!!!  Found only 1 reflection!");
			visionData.setResult(false);
			return visionData;
		}
		else if (rectList.size() == 2)
		{  
			if (rectList.get(0).x < rectList.get(1).x)
			{
				left = rectList.get(0);
				right = rectList.get(1); 
			}
			else
			{
				left = rectList.get(1);
				right = rectList.get(0); 
			}
		}
		else
		{
			System.out.println("Too many reflections found.  Reducing to two.");
			double big = 0, bigger = 0, biggest = 0;
			int big_i = 0, bigger_i = 0, biggest_i = 0;
			int i = 0;
			Double area1, area2; 
			for (Rect rr : rectList)
			{
				if (rr.area() > biggest)
				{
					big = bigger; bigger = biggest; biggest = rr.area();
					big_i = bigger_i; bigger_i = biggest_i; biggest_i = i;    			  
				}
				else if (rr.area() > bigger)
				{
					big = bigger; bigger = rr.area();
					big_i = bigger_i; bigger_i = i;
				}
				else if (rr.area() > big)
				{
					big = rr.area();
					big_i = i;
				}

				i++;
			}

			System.out.println("Big  Bigger  Biggest: " + 
					big_i + "  " + bigger_i + "  " + biggest_i);
			taller = rectList.get(biggest_i);
			left = (rectList.get(biggest_i).x < rectList.get(bigger_i).x) ? 
					rectList.get(biggest_i) : rectList.get(bigger_i);
					right = (rectList.get(biggest_i).x >= rectList.get(bigger_i).x) ? 
							rectList.get(biggest_i) : rectList.get(bigger_i);    
		}

		// Find the center pixels of the left and right tape
		int center_of_left_tape = (left.x + (left.width / 2)); 
		int center_of_right_tape = (right.x + (right.width / 2));

		// Calculate the distance based on the center point of the 2 rectangles
		int center_to_center_dist = (center_of_right_tape - center_of_left_tape);

		if (center_to_center_dist <= (0.03 * res_x)) {  // if dist is farther than, say, ~250"

			System.out.println("ERROR: Center to Center distance of reflective tape is less than 10 pixels.");       	
			visionData.setResult(false);
			return visionData;
		}

		distance_in_inches = ( (int) ((360 * 8.5 * pixels_per_degree_X) / (center_to_center_dist * 2 * 3.14)) );
		// Subtract fixed distance from Tape to tip of Pin
		//        distance_in_inches = (int)(distance_in_inches - Enums.PEG_LENGTH 
		//        		- Enums.PEG_CHANNEL_DEPTH - Enums.CAMERA_DIST_FROM_FRONT);
		distance_in_inches = (int)(distance_in_inches - Enums.PEG_LENGTH - Enums.PEG_CHANNEL_DEPTH);  // better results

		visionData.setDistanceInInches((int) distance_in_inches);

		// Calculate the Facing Angle based on center pixel of tape compared to center of image captured by camera
		int center_pixel_between_tape = (center_of_left_tape + (center_to_center_dist / 2));
		int center_pixel_of_camera = res_x / 2;
		int offset_to_center_of_camera = center_pixel_between_tape - center_pixel_of_camera;
		facing_angle_in_deg = offset_to_center_of_camera / pixels_per_degree_X;

		if (facing_angle_in_deg < 0)
		{
			System.out.println("Robot has to be rotated left...");       	
		}
		else
		{
			System.out.println("Robot has to be rotated right...");       	
		}

		visionData.setFacingAngleInDeg(facing_angle_in_deg);

		// Compute the ratio of the width of the two main rectangles.
		// Because the peg delivery channel may obstruct the full view of 
		// both tape/rect's when the robot is off the centerline by > N degrees (TBD), 
		// this computation *should* be related to our desired "AngleFromCenterline".
		// We will make a convention that the ratio is positive when the angle is 
		// counterclockwise from the centerline, negative otherwise.
		double rw = right.width; double lw = left.width;
		double widthRatio = (rw > lw) ? lw / rw : rw / lw;

		widthRatio = (rw > lw) ? widthRatio : -widthRatio;
		System.out.println("\nLEFT width = " + left.width + ", RIGHT width = " + right.width + 
				", Width ratio = " + widthRatio);

		// Compute the ratio of the area of the two main rectangles.
		double ra = right.area(); double la = left.area();
		double areaRatio = (ra > la) ? la / ra : ra / la;

		areaRatio = (ra > la) ? areaRatio : - areaRatio;
		System.out.println("LEFT area = " + left.area() + ", RIGHT area = " + right.area() + 
				", Area ratio = " + areaRatio);

		// Determine which zone we're in, use the area ratio
		// if areas are close (>= 70 percent), then zone 1
		if ((Math.abs(areaRatio)) >= 0.95)
		{
			zone = 0;
		}
		else if ((Math.abs(areaRatio)) <= 0.7)
		{
			zone = 2;
		}
		else
		{
			zone = 1;
		}

		// if right area larger than left area, then left rectangle is occluded
		// or barely smaller by parallax so right of center and positive zone
		zone = (ra > la) ? zone : -zone;

		visionData.setZone(zone);

		// Rotate robot right/left ?? degrees
		// Move forward ?? inches to centerline
		// Rotate robot 90 degrees to the left/right

		visionData.setResult(true);

		long t1 = System.currentTimeMillis();
		//        System.out.println("\nImage used: " + imageName);
		System.out.println("Distance to target: " + distance_in_inches + " inches");
		System.out.println("Facing angle to target: " + facing_angle_in_deg + " degrees");
		System.out.println("Zone: " + zone);
		System.out.println("Angle to center line: " + (60 - facing_angle_in_deg) + " degrees");
		System.out.println("Done in " + (t1-t0) + " ms");

		return visionData;
	}
}
