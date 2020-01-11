package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.commands.RiseToHeightCommand;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class climbTest {
    Climber climber;
    RiseToHeightCommand rise;

    @Before
    public void setClimber() throws Exception {
        climber = new Climber();
        rise = new RiseToHeightCommand(climber, 2);
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
        Assert.assertEquals(climber.normalizeSetPoint(0), 0,0);
        Assert.assertEquals(2, climber.normalizeSetPoint(2.3),0);
        Assert.assertEquals(1.5, climber.normalizeSetPoint(1.5),0);
    }


}
