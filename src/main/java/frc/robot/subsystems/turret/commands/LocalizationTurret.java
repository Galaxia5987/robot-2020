package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.Utils;

public class LocalizationTurret extends CommandBase {
    private final Turret turret;
    private final Drivetrain drivetrain;

    public LocalizationTurret(Turret turret, Drivetrain drivetrain) {
        addRequirements(turret);
        this.turret = turret;
        this.drivetrain = drivetrain;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(Utils.calculateTurretAngle(drivetrain.getPose(), false));
    }

}
