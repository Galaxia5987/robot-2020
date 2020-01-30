package frc.robot.subsystems.turret;

import frc.robot.subsystems.UnitModel;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static frc.robot.Constants.Turret.TICKS_PER_DEGREE;

public class TurretTest {
    Turret turret;
    UnitModel unitModel;

    @Before
    public void setUp(){
        turret = new Turret();
        unitModel = new UnitModel(TICKS_PER_DEGREE);
    }

    @Test
    public void getNearestTurretPosition() {
        double targetAngle = 280;
        double currentPosition = 30;
        double minPos = -200;
        double maxPos = 200;
        Assert.assertEquals(turret.getNearestTurretPosition(targetAngle, currentPosition, minPos, maxPos), -80, 0.1);
        Assert.assertEquals(turret.getNearestTurretPosition(-359, 200, minPos, maxPos), 1, 0.1);
        Assert.assertEquals(turret.getNearestTurretPosition(300, -40, minPos, maxPos),-60, 0.1);
        Assert.assertEquals(turret.getNearestTurretPosition(20, 320, minPos, maxPos), 20, 0.1);
        Assert.assertEquals(turret.getNearestTurretPosition(320, 20, minPos, maxPos), -40, 0.1);
        Assert.assertEquals(turret.getNearestTurretPosition(173, -247, minPos, maxPos), -187, 0.1);
    }


    @Test
    public void center() {
        double currentPos = 200;
        double minPos = -360;
        double maxPos = 360;
        Assert.assertEquals(turret.center(currentPos, minPos, maxPos), -160, 0.1);
        Assert.assertEquals(turret.center(-20, minPos, maxPos), -20, 0.1);
        Assert.assertEquals(turret.center(-180, minPos, maxPos), -180, 0.1);
        Assert.assertEquals(turret.center(-200, minPos, maxPos),160, 0.1);
        Assert.assertEquals(turret.center(50, minPos, maxPos),50, 0.1);
    }
}