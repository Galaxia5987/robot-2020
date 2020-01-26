package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class TurnLocalization extends CommandBase {
    private final Turret turret;

    public TurnLocalization(Turret turret) {
        this.turret = turret;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(calculateTargetAngle());
    }

    private double calculateTargetAngle() {
        Pose2d localization = turret.getLocalization();
        double deltaY = Math.abs(Constants.Field_Geometry.POWER_PORT_LOCATION.getTranslation().getY() - localization.getTranslation().getY());
        double deltaX = Math.abs(Constants.Field_Geometry.POWER_PORT_LOCATION.getTranslation().getX() - localization.getTranslation().getX());
        return -1 * (localization.getRotation().getRadians() - Math.atan2(deltaY, deltaX));
    }
}
