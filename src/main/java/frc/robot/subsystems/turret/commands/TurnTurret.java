package frc.robot.subsystems.turret.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;

/**
 * this command turns the turret until it reaches a certain threshold.
 */
public class TurnTurret extends CommandBase {

    private static Turret turret;
    private final double angle;

    public TurnTurret(Turret turret, double angle) {
        TurnTurret.turret = turret;
        addRequirements(turret);
        this.angle = angle;
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
        return Math.abs(turret.getEncoderPosition() - angle) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop(angle);
    }
}