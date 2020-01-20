package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.valuetuner.WebConstant;

/**
 *
 */
public class VelocityDrive extends CommandBase {
    private Drivetrain drivetrain;
    private WebConstant desiredLeftVelocity = new WebConstant("desiredLeftVelocity", 1);
    private WebConstant desiredRightVelocity = new WebConstant("desiredRightVelocity", 1);

    public VelocityDrive(Drivetrain drivetrain) {
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true

    public void end(boolean interrupted) {
    }

}