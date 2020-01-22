package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Shooter.VELOCITY_THRESHOLD;
import static frc.robot.Constants.Turret.ANGLE_THRESHOLD;

public class WaitForShootingVision extends CommandBase {
    private Shooter shooter;
    private Turret turret;
    private double distance;
    private double angle;

    public WaitForShootingVision(Shooter shooter, Turret turret) {
        addRequirements(shooter, turret);
        this.shooter = shooter;
        this.turret = turret;
        this.distance = shooter.getVisionDistance();
        this.angle = turret.getVisionAngle();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        boolean isShooterReady = Math.abs(shooter.getSpeed() - shooter.approximateVelocity(distance)) >= VELOCITY_THRESHOLD;
        boolean isTurretReady = Math.abs(turret.getEncoderPosition() - angle) <= ANGLE_THRESHOLD;
        return isShooterReady && isTurretReady;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
