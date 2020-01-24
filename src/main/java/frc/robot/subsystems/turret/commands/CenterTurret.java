package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;

public class CenterTurret extends CommandBase {
    private double initialPosition;
    private static Turret turret;

    public CenterTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        initialPosition = turret.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(turret.center(turret.getAngle(), MINIMUM_POSITION, MAXIMUM_POSITION));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(turret.getAngle() - initialPosition) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
