package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;


public class WaitForShootingVision extends CommandBase {
    private Shooter shooter;
    private Turret turret;

    public WaitForShootingVision(Shooter shooter, Turret turret) { //This reads the values, so there is no reason to require.
        addRequirements(shooter, turret);
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
        return shooter.isShooterReady() && turret.isTurretReady();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
