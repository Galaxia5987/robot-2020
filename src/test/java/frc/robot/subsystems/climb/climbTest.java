package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.commands.BalanceRobot;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static frc.robot.RobotContainer.climber;

public class climbTest {
    BalanceRobot rise;

    @Before
    public void setClimber() throws Exception {
        rise = new BalanceRobot(climber, 2);
    }

    @Test
    public void normalizeHeightsTest() {
        Assert.assertTrue(rise.normalizeHeights(0.4, 1, 1, 1, 3)[0] >= 1);
        Assert.assertEquals(0.6, rise.normalizeHeights(0.4, 1, 1, 0, 3)[0], 0.0);
        Assert.assertEquals(1.4, rise.normalizeHeights(1.4, 1, 1, 0, 2)[1], 0.0);
        Assert.assertEquals(2, rise.normalizeHeights(2, 1, 1, 0, 2)[1], 0.0);
        Assert.assertEquals(2, rise.normalizeHeights(2, 1, 2, 1, 2)[1], 0.0);
        Assert.assertEquals(1, rise.normalizeHeights(2, 1, 2, 1, 2)[0], 0.0);
    }

    @Test
    public void normalizeSetpointTest() {
        Assert.assertEquals(climber.normalizeSetpoint(0), 0,0);
        Assert.assertEquals(2, climber.normalizeSetpoint(2.3),0);
        Assert.assertEquals(1.5, climber.normalizeSetpoint(1.5),0);
    }
}
