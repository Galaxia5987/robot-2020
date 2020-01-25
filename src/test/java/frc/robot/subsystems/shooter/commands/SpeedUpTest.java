package frc.robot.subsystems.shooter.commands;


import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import org.junit.Assert;
import org.junit.Test;


public class SpeedUpTest {
    private final double distance = 2;
    private Shooter shooter = new Shooter();
    private Conveyor conveyor = new Conveyor();
    private SpeedUp speedUp = new SpeedUp(shooter, conveyor, distance);

    @Test
    public void approximateVelocity() {
        Assert.assertEquals(shooter.approximateVelocity(distance), 12.15, 1e-2);
    }
}
