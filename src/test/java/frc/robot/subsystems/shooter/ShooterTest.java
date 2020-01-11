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
        int ticks = 12;
        double desired_RPM = 1.2;
        double RPM_res = shooter.ticksToRPM(12);

//        Assert.assertEquals (desired_RPM, RPM_res,1e-10);
    }

    @Test
    public void RPMToTicks() {
    }
}
