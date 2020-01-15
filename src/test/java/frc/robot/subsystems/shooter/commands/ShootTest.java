package frc.robot.subsystems.shooter.commands;


import frc.robot.subsystems.shooter.Shooter;
import org.junit.Assert;
import org.junit.Test;


public class ShootTest {
    private final double distance = 2;
    private Shooter shooter = new Shooter();
    private Shoot shoot = new Shoot(shooter, distance);

    @Test
    public void approximateVelocity() {
        Assert.assertEquals(shooter.approximateVelocity(distance), 12.15, 1e-2);
    }
}
