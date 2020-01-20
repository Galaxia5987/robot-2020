package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

/**
 *
 */
public class VelocityDrive extends CommandBase {
    private Drivetrain drivetrain;
    private WebConstant desiredLeftVelocity = new WebConstant("desiredLeftVelocity", 1);
    private WebConstant desiredRightVelocity = new WebConstant("desiredRightVelocity", 1);

    public VelocityDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        drivetrain.setVelocityAndFeedForward(desiredLeftVelocity.get(), desiredRightVelocity.get(), 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (Robot.debug) {
            FireLog.log("driveLeftSetpoint", desiredLeftVelocity);
            FireLog.log("driveRightSetpoint", desiredRightVelocity);
            FireLog.log("rightVelocity", drivetrain.getRightVelocity());
            FireLog.log("leftVelocity", drivetrain.getLeftVelocity());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true

    public void end(boolean interrupted) {
        drivetrain.setVelocityAndFeedForward(0, 0, 0, 0);
    }

}