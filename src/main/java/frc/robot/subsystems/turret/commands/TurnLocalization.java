package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.FieldGeometry.INNER_POWER_PORT_LOCATION;

public class TurnLocalization extends CommandBase {
    private final Turret turret;
    private final Drivetrain drivetrain;

    public TurnLocalization(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(calculateTargetAngle(drivetrain.getPose()));
    }

    public double calculateTargetAngle(Pose2d currentPosition) {
        double deltaY = INNER_POWER_PORT_LOCATION.getTranslation().getY() - currentPosition.getTranslation().getY();
        double deltaX = INNER_POWER_PORT_LOCATION.getTranslation().getX() - currentPosition.getTranslation().getX();
        return Math.toDegrees(Math.atan2(deltaY, deltaX) - currentPosition.getRotation().getRadians());
    }
}
