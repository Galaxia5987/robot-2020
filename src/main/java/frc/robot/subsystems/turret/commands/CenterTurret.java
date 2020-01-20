package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;

public class CenterTurret extends CommandBase {
    private double initialPosition;
    private static Turret turret;

    public CenterTurret(Turret turret) {
        CenterTurret.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        initialPosition = turret.getEncoderPosition();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.center(turret.getEncoderPosition(), MINIMUM_POSITION, MAXIMUM_POSITION);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(turret.getEncoderPosition() - initialPosition) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}