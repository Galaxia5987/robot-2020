package frc.robot.subsystems.shooter;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;


public class ShooterTest {
    private Shooter shooter;

    @Before
    public void createShooterInstance() {
        shooter = new Shooter();
    }


    @Test
    public void ticksToRPMTest() {
        int ticks = 4320;
        double desired_RPM = 72000;
        double gearRatio = 1;
        double RPM_res = shooter.ticksToRPMTest(4320, gearRatio);

        Assert.assertEquals (desired_RPM, RPM_res,1e-10);
    }

    @Test
    public void RPMToTicksTest() {
        int rpm = 72000;
        double desired_ticks = 4320;
        double gearRatio = 1;
        double ticks_res = shooter.RPMToTicksTest(rpm, gearRatio);

        Assert.assertEquals(desired_ticks, ticks_res, 1e-10);
    }
}
