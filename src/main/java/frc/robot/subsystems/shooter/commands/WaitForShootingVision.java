package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Shooter.VELOCITY_TOLERANCE;
import static frc.robot.Constants.Turret.ANGLE_THRESHOLD;

public class WaitForShootingVision extends CommandBase {
    private Shooter shooter;
    private Turret turret;

    public WaitForShootingVision(Shooter shooter, Turret turret) { //This reads the values, so there is no reason to require.
        this.shooter = shooter;
        this.turret = turret;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.approximateVelocity(shooter.getVisionDistance());

    }

    @Override
    public boolean isFinished() {
        boolean isShooterReady = Math.abs(shooter.getSpeed() - shooter.getTargetVelocity()) <= VELOCITY_TOLERANCE;
        boolean isTurretReady = Math.abs(turret.getAngle() - turret.getVisionAngle()) <= ANGLE_THRESHOLD;
        return isShooterReady && isTurretReady;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
