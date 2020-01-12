package frc.robot.subsystems.shooter.commands;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;

import static org.junit.Assert.*;
public class ShootTest {
    Shoot shoot = new Shoot(10);
    @Test
    public void calculateVelocity() {
        System.out.println(  shoot.calculateVelocity(50,80,4.2,1.1,0.4));
        Assert.assertEquals(shoot.calculateVelocity(50,80,4.2,1.1,0.4), 30, 1e-10);
    }
}
