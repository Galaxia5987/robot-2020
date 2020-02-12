package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class VisionTurret extends CommandBase {
    private final Turret turret;

    public VisionTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        VisionModule.setLeds(true);
    }

    @Override
    public void execute() {
        turret.setAngle(turret.getAngle() + VisionModule.getVisionAngle());
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLeds(false);
    }
}
