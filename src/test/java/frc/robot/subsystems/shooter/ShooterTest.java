package frc.robot.subsystems.shooter;

import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import static frc.robot.Constants.Shooter.TICKS_PER_ROTATION;


public class ShooterTest {
    private UnitModel rpsUnitModel = new UnitModel(TICKS_PER_ROTATION);

    @Before
    public void createShooterInstance() {
        Robot.shooter = new Shooter();
    }
    
}
