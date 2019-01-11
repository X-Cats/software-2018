package org.usfirst.frc.xcats.robot;

import org.junit.Assert;
import org.junit.Test;

public class TestClimber extends BaseTest
{

    @Test
    public void testClimber()
    {

        Climber climber = new Climber();
        Assert.assertTrue(climber.getClimbSpeed() == 0);

        climber.climb();
        Assert.assertTrue(climber.getClimbSpeed() > 0);
    }

}
