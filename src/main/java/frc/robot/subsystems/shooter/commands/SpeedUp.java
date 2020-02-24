package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

public class SpeedUp extends CommandBase {
    private Shooter shooter;
    private Drivetrain drivetrain;
    private NetworkTable velocityTable = NetworkTableInstance.getDefault().getTable("velocityTable");
    private final NetworkTableEntry velocityEntry = velocityTable.getEntry("velocity");
    private double distance;
    private boolean isVisionActive = false;
    private boolean end = true;

    public SpeedUp(Shooter shooter, double distance) {
        addRequirements(shooter);
        this.distance = distance;
        this.shooter = shooter;
        this.drivetrain = null;
    }

    public SpeedUp(Shooter shooter, Drivetrain drivetrain) {
        this(shooter, true, drivetrain);
    }

    public SpeedUp(Shooter shooter, boolean end, Drivetrain drivetrain) {
        addRequirements(shooter);
        isVisionActive = true;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.end = end;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (isVisionActive && VisionModule.getHoodDistance() != null) {
            distance = VisionModule.getHoodDistance();
        } else if (isVisionActive) {
            if (!VisionModule.targetSeen())
                distance = Utils.localizationDistanceToPort(drivetrain.getPose());
        }
        shooter.setSpeed(shooter.approximateVelocity(distance));
        SmartDashboard.putNumber("aproximateVelocity", shooter.approximateVelocity(distance));
        SmartDashboard.putNumber("wallDistance", distance);

        velocityEntry.setDouble(shooter.getSpeed());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
        if (this.end)
            shooter.stop();
    }

}
