package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Robot.shooter;

public class Shoot extends CommandBase {
    private double distance;
    private double timeout;
    private Timer timer = new Timer();
    public static NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private NetworkTableEntry velocityEntry = shooterTable.getEntry("velocity");
    private UnitModel unitModel = new UnitModel(TICKS_PER_SECOND);

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
        shooter.setSpeedRPM(approximateVelocity(distance));
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
     * @return the calculated velocity to get to the target.
     */
    private double approximateVelocity(double distance) {
        return (520.78 * Math.exp(0.1685 * distance));
    }

    /**
     * @return return the velocity that is needed to reach the target
     */
    private double calculateVelocity() {
        double velocity = Math.sqrt((-g*Math.pow(TARGET_DISTANCE, 2))/2*Math.pow(Math.cos(Math.toRadians(ANGLE)),2)*(TARGET_HEIGHT-SHOOTER_HEIGHT-TARGET_DISTANCE*Math.tan(Math.toRadians(ANGLE))));
        return convertMPSToRPM(velocity * VELOCITY_DIFFERENCE);
    }

    /**
     *
     * @param mps
     * @return the conversion between mps and rpm
     */
    private double convertMPSToRPM(double mps) {
        return unitModel.toUnits(mps)/60;
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
        shooter.setSpeedRPM(0);
    }

    // Called when another command which requires one or more of the same
// subsystems is scheduled to run
    protected void interrupted() {
    }
}
