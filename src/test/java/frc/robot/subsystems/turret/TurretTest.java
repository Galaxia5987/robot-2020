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
    public void setTurretAngle() {
        double targetAngle = 280;
        Assert.assertEquals(turret.setTurretAngle(targetAngle), turret.convertDegreesToTicks(-80), 0.1);
        Assert.assertEquals(turret.setTurretAngle(-359), turret.convertDegreesToTicks(1), 0.1);
        Assert.assertEquals(turret.setTurretAngle(300), turret.convertDegreesToTicks(-60), 0.1);
        Assert.assertEquals(turret.setTurretAngle(20), turret.convertDegreesToTicks(20), 0.1);
        Assert.assertEquals(turret.setTurretAngle(320), turret.convertDegreesToTicks(-40), 0.1);
        Assert.assertEquals(turret.setTurretAngle(173), turret.convertDegreesToTicks(-187), 0.1);
    }


    @Test
    public void center() {
        Assert.assertEquals(turret.center(), turret.convertDegreesToTicks(-80), 0.1);
    }
}