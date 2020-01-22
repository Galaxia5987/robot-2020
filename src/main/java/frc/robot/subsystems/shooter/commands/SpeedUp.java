package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class SpeedUp extends CommandBase {
    public static final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private static final NetworkTableEntry visionDistance = shooterTable.getEntry("distance");
    private final Shooter shooter;
    private final NetworkTableEntry velocityEntry = shooterTable.getEntry("velocity");
    private double distance;
    private boolean isVisionActive = false;
    private boolean stop = true;

    public SpeedUp(Shooter shooter, double distance) {
        addRequirements(shooter);
        this.distance = distance;
        this.shooter = shooter;
    }


    public SpeedUp(Shooter shooter) {
        this(shooter, visionDistance.getDouble(0)); //TODO replace 3 with the vision distance output value
        isVisionActive = true;
    }

    public SpeedUp(Shooter shooter, boolean stop) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.stop = stop;
        this.distance = visionDistance.getDouble(0);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (isVisionActive) {
            distance = visionDistance.getDouble(0);
        }
        shooter.setSpeed(shooter.approximateVelocity(distance));
        setNetworkTable();
    }

    /**
     * sets the velocity in the network table
     */
    private void setNetworkTable() {
        velocityEntry.setDouble(shooter.getSpeed());
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return stop; //TODO: Check if timeout == 0 works instead, depending on how whileHeld works.
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }

}
