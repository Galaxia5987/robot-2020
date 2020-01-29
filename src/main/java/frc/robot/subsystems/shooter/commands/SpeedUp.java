package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

public class SpeedUp extends CommandBase {
    private NetworkTable velocityTable = NetworkTableInstance.getDefault().getTable("velocityTable");
    private final NetworkTableEntry velocityEntry = velocityTable.getEntry("velocity");
    private final Shooter shooter;
    private double distance;

    public SpeedUp(Shooter shooter, double distance) {
        addRequirements(shooter);
        this.distance = distance;
        this.shooter = shooter;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        shooter.approximateVelocity(distance);
        shooter.setSpeed(shooter.getTargetVelocity());
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
        shooter.stop();
    }

}
