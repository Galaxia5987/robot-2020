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
        Assert.assertEquals(shoot.approximateVelocity(distance), 20, 1e-3);
    }
}
