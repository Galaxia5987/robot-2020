package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
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
        Pose2d visionPose = VisionModule.getPose();
        if(visionPose == null) return;
        VisionModule.calculateTargetAngle(visionPose.relativeTo(OUTER_POWER_PORT_LOCATION), true);
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
