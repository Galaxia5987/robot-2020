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
    public void ticksToRPM() {
        int ticks = 4320;
        double desired_RPM = 12;
        double RPM_res = shooter.ticksToRPM(4320);

        Assert.assertEquals (desired_RPM, RPM_res,1e-10);
    }

    @Test
    public void RPMToTicks() {
        int rpm = 12;
        double desired_ticks = 4320;
        double ticks_res = shooter.RPMToTicks(12);

        Assert.assertEquals(desired_ticks, ticks_res, 1e-10);
    }
}
