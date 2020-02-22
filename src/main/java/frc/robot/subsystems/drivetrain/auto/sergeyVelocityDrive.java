package frc.robot.subsystems.drivetrain.auto;

import edu.wpi.first.wpilibj.Timer;
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
public class sergeyVelocityDrive extends CommandBase {
    private final boolean usePid;
    private final boolean useFF;
    private final boolean reverse;
    private Drivetrain drivetrain;
    private static final SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.leftkS, Constants.Autonomous.leftkV, Constants.Autonomous.leftkA);
    private static final SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.rightkS, Constants.Autonomous.rightkV, Constants.Autonomous.rightkA);
    private static final WebConstant desiredVelocity = new WebConstant("desiredVelocity", 1);
    private static final WebConstant desiredVelocitySecondInterval = new WebConstant("desiredVelocitySecondInterval", 0.5);
    private Timer intervalTimer = new Timer();
    private static double intervalTimeout = 2;
    private double velocity2;
    private double velocity;


    public sergeyVelocityDrive(Drivetrain drivetrain, boolean usePid, boolean useFF, boolean reverse) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.usePid = usePid;
        this.useFF = useFF;
        this.reverse = reverse;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        intervalTimer.reset();
        intervalTimer.start();
        this.velocity = desiredVelocity.get();
        this.velocity2 = desiredVelocitySecondInterval.get();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (intervalTimer.get() > intervalTimeout){
            intervalTimer.reset();
            velocity = velocity == desiredVelocity.get() ? velocity2 : velocity;
        }

        if(reverse) {
            velocity = -velocity;
        }

        FireLog.log("driveVelocitySetpoint", Math.abs(velocity));

        double leftFeedforward =
                leftfeedforward.calculate(velocity, 0);

        double rightFeedforward =
                rightfeedforward.calculate(velocity, 0);

        System.out.println(leftFeedforward / 12);

        if(!useFF) {
            leftFeedforward = 0;
            rightFeedforward = 0;
        }

        if (usePid)
            drivetrain.setVelocityAndFeedForward(velocity, velocity, leftFeedforward / 12, rightFeedforward / 12);
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