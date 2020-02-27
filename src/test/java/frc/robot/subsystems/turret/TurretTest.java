package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.UnitModel;
import frc.robot.subsystems.turret.commands.LocalizationTurret;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;
import org.junit.Assert;
import org.junit.Before;

import static frc.robot.Constants.Turret.TICKS_PER_DEGREE;

public class TurretTest {
    Turret turret;
    UnitModel unitModel;

    @Before
    public void setUp() {
        turret = new Turret();
        unitModel = new UnitModel(TICKS_PER_DEGREE);
    }

    //@Test
    public void TurretLocalization() {
        // Inner port tests
//        Assert.assertEquals(turnLocalization.calculateTargetAngle(new Pose2d(7, 4, new Rotation2d(Math.toRadians(56.4)))), -45.893, 0.1);
//        Assert.assertEquals(turnLocalization.calculateTargetAngle(new Pose2d(3.5, 0.2, new Rotation2d(Math.toRadians(29.4)))), -6.4678, 0.1);
//        Assert.assertEquals(turnLocalization.calculateTargetAngle(new Pose2d(3, 0, new Rotation2d(Math.toRadians(20.7)))), 2.19133, 0.1);
//        Assert.assertEquals(turnLocalization.calculateTargetAngle(new Pose2d(3.5, 0.2, new Rotation2d(Math.toRadians(-20)))), 42.932, 0.1);

        // Outer port tests
        Assert.assertEquals(Utils.calculateTurretAngle(new Pose2d(7, 4, new Rotation2d(Math.toRadians(56.4))), false), -45, 0.1);
        Assert.assertEquals(Utils.calculateTurretAngle(new Pose2d(3.5, 0.2, new Rotation2d(Math.toRadians(29.4))), false), -5.195, 0.1);
        Assert.assertEquals(Utils.calculateTurretAngle(new Pose2d(3, 0, new Rotation2d(Math.toRadians(20.7))), false), 3.4138, 0.1);
        Assert.assertEquals(Utils.calculateTurretAngle(new Pose2d(3.5, 0.2, new Rotation2d(Math.toRadians(-20))), false), 44.204, 0.1);
    }
}