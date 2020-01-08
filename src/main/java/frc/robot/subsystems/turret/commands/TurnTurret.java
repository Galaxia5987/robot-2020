package frc.robot.subsystems.turret.commands;


import edu.wpi.first.wpilibj2.command.*;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Robot.turret;

/**
 *
 */
public class TurnTurret extends CommandBase {

    private double angle;

    public TurnTurret(double angle) {
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
        turret.moveTurret(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(turret.getAngle() - angle) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}