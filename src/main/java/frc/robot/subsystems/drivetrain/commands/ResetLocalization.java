package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class ResetLocalization extends CommandBase {

    public ResetLocalization(Drivetrain drivetrain) {
        drivetrain.setPose(VisionModule.getRobotPose());
    }

}
