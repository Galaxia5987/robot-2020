package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

import static frc.robot.Constants.FieldGeometry.OUTER_POWER_PORT_LOCATION;

public class PoseVisionTurret extends CommandBase {
    private final Turret turret;

    public PoseVisionTurret(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    @Override
    public void execute() {
        Pose2d robotPose = VisionModule.getRobotPose();
        if(robotPose == null || turret.inDeadZone()) return;
        turret.setAngle(Utils.calculateTurretAngle(robotPose, true));
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
