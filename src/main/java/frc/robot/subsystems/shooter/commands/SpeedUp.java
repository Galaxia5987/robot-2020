package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class SpeedUp extends CommandBase {
    public static final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private double distance;
    private double timeout;
    private final Shooter shooter;
    private final Timer timer = new Timer();
    private final NetworkTableEntry velocityEntry = shooterTable.getEntry("velocity");
    private static final NetworkTableEntry visionDistance = shooterTable.getEntry("distance");
    private boolean isVisionActive = false;
    private boolean stop = false;

    public SpeedUp(Shooter shooter, double distance, double timeout) {
        addRequirements(shooter);
        this.distance = distance;
        this.timeout = timeout;
        this.shooter = shooter;
    }

    public SpeedUp(Shooter shooter, double timeout) {
        this(shooter, visionDistance.getDouble(3), timeout);
    }

    public SpeedUp(Shooter shooter) {
        this(shooter, visionDistance.getDouble(3), 0); //TODO replace 3 with the vision distance output value
        isVisionActive = true;
    }

    public SpeedUp(Shooter shooter, boolean stop) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.stop = stop;
        this.distance = visionDistance.getDouble(3);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (isVisionActive) {
            distance = visionDistance.getDouble(3);
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
        return (timeout == 0 || timer.get() >= timeout) && stop; //TODO: Check if timeout == 0 works instead, depending on how whileHeld works.
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setSpeed(0);
    }

}
