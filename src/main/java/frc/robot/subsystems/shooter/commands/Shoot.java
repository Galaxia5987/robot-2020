package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends CommandBase {
    public static final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private double distance;
    private final double timeout;
    private final Shooter shooter;
    private final Timer timer = new Timer();
    private final NetworkTableEntry velocityEntry = shooterTable.getEntry("velocity");
    private static final NetworkTableEntry visionDistance = shooterTable.getEntry("distance");
    private boolean isVisionActive = false;

    public Shoot(Shooter shooter,  double distance, double timeout) {
        addRequirements(shooter);
        this.distance = distance;
        this.timeout = timeout;
        this.shooter = shooter;
    }

    public Shoot(Shooter shooter, double distance) {
        this(shooter, distance, 0);
    }

    public Shoot(Shooter shooter) {
        this(shooter, visionDistance.getDouble(3), 0); //TODO replace 3 with the vision distance output value
        isVisionActive = true;
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
        return timeout > 0 && timer.get() >= timeout; //TODO: Check if timeout == 0 works instead, depending on how whileHeld works.
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setSpeed(0);
    }

}
