package frc.robot.subsystems.turret;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class TurretTest {
    Turret turret;

    @Before
    public void setUp() throws Exception {
        turret = new Turret();
    }

    @Test
    public void getCorrectionPosition() {
        double targetAngle = 280;
        double currentPosition = 30;
        double minPos = -360;
        double maxPos = 360;
        Assert.assertEquals(turret.getCorrectPosition(targetAngle, currentPosition, minPos, maxPos), turret.convertDegreesToTicks(-80), 0.1);
        Assert.assertEquals(turret.getCorrectPosition(-359, 200, minPos, maxPos), turret.convertDegreesToTicks(1), 0.1);
        Assert.assertEquals(turret.getCorrectPosition(300, -40, minPos, maxPos), turret.convertDegreesToTicks(-60), 0.1);
        Assert.assertEquals(turret.getCorrectPosition(20, 320, minPos, maxPos), turret.convertDegreesToTicks(20), 0.1);
        Assert.assertEquals(turret.getCorrectPosition(320, 20, minPos, maxPos), turret.convertDegreesToTicks(-40), 0.1);
        Assert.assertEquals(turret.getCorrectPosition(173, -247, minPos, maxPos), turret.convertDegreesToTicks(-187), 0.1);
    }


    @Test
    public void getCorrectCenterPosition() {
        double currentPos = 200;
        double minPos = -360;
        double maxPos = 360;
        Assert.assertEquals(turret.getCorrectCenterPosition(currentPos, minPos, maxPos), turret.convertDegreesToTicks(-160), 0.1);
    }
}