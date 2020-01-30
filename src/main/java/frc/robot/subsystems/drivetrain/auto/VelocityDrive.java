package frc.robot.subsystems.drivetrain.auto;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
    private static final SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.leftkS, Constants.Autonomous.leftkV, Constants.Autonomous.leftkA);
    private static final SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.rightkS, Constants.Autonomous.rightkV, Constants.Autonomous.rightkA);
    private static final WebConstant desiredVelocity = new WebConstant("desiredVelocity", 1);

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
            FireLog.log("driveVelocitySetpoint", Math.abs(desiredVelocity.get()));
            FireLog.log("rightVelocity", Math.abs(drivetrain.getRightVelocity()));
            FireLog.log("leftVelocity", Math.abs(drivetrain.getLeftVelocity()));
        }


        double leftFeedforward =
                leftfeedforward.calculate(desiredVelocity.get(), 0);

        double rightFeedforward =
                rightfeedforward.calculate(desiredVelocity.get(), 0);

        System.out.println(leftFeedforward / 12);

        if (!useFF) {
            leftFeedforward = 0;
            rightFeedforward = 0;
        }

        if (usePid)
            drivetrain.setVelocityAndFeedForward(desiredVelocity.get(), desiredVelocity.get(), leftFeedforward / 12, rightFeedforward / 12);
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