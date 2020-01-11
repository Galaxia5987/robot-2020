package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Robot.turret;

public class CenterTurret extends CommandBase {
    private double currentPosition;

    public CenterTurret() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        currentPosition = turret.getEncoderPosition();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.center();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(turret.getEncoderPosition() - currentPosition) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
