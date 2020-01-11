package frc.robot.subsystems.turret;

import org.junit.Assert;
import org.junit.Test;


public class TurretAlgorithmsTest {

    private TurretAlgorithms algorithmsTest = new TurretAlgorithms();

    @Test
    public void setTurretAngle() {
        double targetAngle = 280;
        double currentPosition = -40;
        double minPos = -359;
        double maxPos = 359;
        Assert.assertEquals(algorithmsTest.setTurretAngle(targetAngle, currentPosition, minPos, maxPos), -80, 0.1);
        Assert.assertEquals(algorithmsTest.setTurretAngle(-359, 200, minPos, maxPos), 1, 0.1);
        Assert.assertEquals(algorithmsTest.setTurretAngle(300, -40, minPos, maxPos), -60, 0.1);
        Assert.assertEquals(algorithmsTest.setTurretAngle(20, 320, minPos, maxPos), 20, 0.1);
        Assert.assertEquals(algorithmsTest.setTurretAngle(320, 20, minPos, maxPos), -40, 0.1);
        Assert.assertEquals(algorithmsTest.setTurretAngle(173, -247, minPos, maxPos), -187, 0.1);
    }


    @Test
    public void center() {
        Assert.assertEquals(algorithmsTest.center(280, -359, 300), -80, 0.1);
    }
}