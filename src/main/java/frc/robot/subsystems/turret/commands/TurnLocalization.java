package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.FieldGeometry.POWER_PORT_LOCATION;

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
        turret.setAngle(calculateTargetAngle());
    }

    private double calculateTargetAngle() {
        Pose2d localization = drivetrain.getPose();
        double deltaY = Math.abs(POWER_PORT_LOCATION.getTranslation().getY() - localization.getTranslation().getY());
        double deltaX = Math.abs(POWER_PORT_LOCATION.getTranslation().getX() - localization.getTranslation().getX());
        return Math.toDegrees(Math.atan2(deltaY, deltaX) - localization.getRotation().getRadians());
    }
}
