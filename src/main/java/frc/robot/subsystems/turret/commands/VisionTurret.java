package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class VisionTurret extends CommandBase {
    private final Turret turret;

    public VisionTurret(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void execute() {
        this.turret.setAngle(turret.getAngle() + turret.getVisionAngle());
    }
}
