package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.FullLocalization;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.FieldGeometry.OUTER_POWER_PORT_LOCATION;

public class TurnLocalization extends CommandBase {
    private final Turret turret;
    private final FullLocalization localization;

    public TurnLocalization(Turret turret, FullLocalization localization) {
        this.turret = turret;
        this.localization = localization;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(calculateTargetAngle(localization.getPose()));
    }

    public static double calculateTargetAngle(Pose2d currentPosition) {
        double deltaY = OUTER_POWER_PORT_LOCATION.getTranslation().getY() - currentPosition.getTranslation().getY();
        double deltaX = OUTER_POWER_PORT_LOCATION.getTranslation().getX() - currentPosition.getTranslation().getX();
        return Math.toDegrees(Math.atan2(deltaY, deltaX) - currentPosition.getRotation().getRadians());
    }
}
