package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Robot.shooter;

public class Shoot extends CommandBase {
    private double distance;
    private double timeout;
    private Timer timer = new Timer();
    public static NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private NetworkTableEntry velocityEntry = shooterTable.getEntry("velocity");

    public Shoot(double distance, double timeout) {
        addRequirements(shooter);
        this.distance = distance;
        this.timeout = timeout;
    }

    public Shoot(double distance) {
        this(distance, 0);
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
        shooter.setSpeedRPS(approximateVelocity(distance));
        setNetworkTable();
    }

    /**
     * sets the velocity in the network table
     */
    private void setNetworkTable() {
        velocityEntry.setDouble(shooter.getSpeed());
    }

    /**
     * @param distance the distance away from the target.
     * @return the calculated velocity to get to the target in rps.
     */
    private double approximateVelocity(double distance) {
        return (8.68 * Math.exp(0.1685 * distance));
    }

    /**
     * @return return the velocity that is needed to reach the target
     */
    private double calculateVelocity(double targetDistance) {
        double velocity =
                Math.sqrt(
                        (-g * targetDistance * targetDistance) /
                                (2 * Math.pow(Math.cos(Math.toRadians(ANGLE)), 2) * (TARGET_HEIGHT - SHOOTER_HEIGHT - targetDistance * Math.tan(Math.toRadians(ANGLE))))
                );
        return convertMPSToRPS(velocity * WHEEL_BALL_VELOCITY_RATIO); //velocity is the balls velocity, but we need to return the wheel velocity.
    }

    /**
     *
     * @param mps
     * @return the conversion between mps and rpm
     */
    private double convertMPSToRPS(double mps) {
        return mps / (2*Math.PI*RADIUS);
    }
    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return timeout > 0 && timer.get() >= timeout;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooter.setSpeedRPS(0);
    }

    // Called when another command which requires one or more of the same
// subsystems is scheduled to run
    protected void interrupted() {
    }
}
