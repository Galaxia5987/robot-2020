package frc.robot.subsystems.turret.commands;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;

/**
 * this command turns the turret until it reaches a certain threshold.
 */
public class TurnTurret extends CommandBase {

    private Turret turret;
    private double angle;
    private boolean stop = true;
    public static final NetworkTable turretTable = NetworkTableInstance.getDefault().getTable("shooter");
    private static final NetworkTableEntry visionAngle = turretTable.getEntry("angle");

    public TurnTurret(Turret turret, double angle) {
        this.turret = turret;
        addRequirements(turret);
        this.angle = angle;
    }

    public TurnTurret(Turret turret){
        this(turret, visionAngle.getDouble(3));
    }

    public TurnTurret(Turret turret, boolean stop){
        addRequirements(turret);
        this.turret = turret;
        this.stop = stop;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(turret.getEncoderPosition() - angle) <= ANGLE_THRESHOLD && stop;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}