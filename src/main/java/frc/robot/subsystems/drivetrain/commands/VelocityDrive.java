package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

/**
 *
 */
public class VelocityDrive extends CommandBase {
    private final boolean usePid;
    private final boolean useFF;
    private Drivetrain drivetrain;
    private WebConstant desiredLeftVelocity = new WebConstant("desiredLeftVelocity", 1);
    private WebConstant desiredRightVelocity = new WebConstant("desiredRightVelocity", 1);
    private WebConstant desiredAcceleration = new WebConstant("desiredAcceleration", 1);
    private static final SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.leftkS, Constants.Autonomous.leftkV, Constants.Autonomous.leftkA);
    private static final SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.rightkS, Constants.Autonomous.rightkV, Constants.Autonomous.rightkA);

    public VelocityDrive(Drivetrain drivetrain, boolean usePid, boolean useFF) {
        this.drivetrain = drivetrain;
        this.usePid = usePid;
        this.useFF = useFF;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (Robot.debug) {
            FireLog.log("driveLeftSetpoint", desiredLeftVelocity.get());
            FireLog.log("driveRightSetpoint", desiredRightVelocity.get());
            FireLog.log("rightVelocity", drivetrain.getRightVelocity());
            FireLog.log("leftVelocity", drivetrain.getLeftVelocity());
            SmartDashboard.putNumber("leftVelocity", drivetrain.getLeftVelocity());
            SmartDashboard.putNumber("rightVelocity", drivetrain.getRightVelocity());
        }


        double leftFeedforward =
                leftfeedforward.calculate(desiredLeftVelocity.get(), 0);

        double rightFeedforward =
                rightfeedforward.calculate(desiredRightVelocity.get(), 0);

        System.out.println(leftFeedforward / 12);

        if(!useFF) {
            leftFeedforward = 0;
            rightFeedforward = 0;
        }

        if (usePid)
            drivetrain.setVelocityAndFeedForward(desiredLeftVelocity.get(), desiredRightVelocity.get(), leftFeedforward / 12, rightFeedforward / 12);
        else {
            drivetrain.setVelocityAndFeedForward(0, 0, leftFeedforward / 12, rightFeedforward / 12);
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