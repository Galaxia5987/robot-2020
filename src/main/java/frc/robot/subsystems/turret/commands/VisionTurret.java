package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class VisionTurret extends CommandBase {
    private final Turret turret;

    public VisionTurret(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        turret.setTalonSlot(1);
        VisionModule.setLeds(true);
    }

    @Override
    public void execute() {
        this.turret.setAnglePosition(turret.getAngle() + VisionModule.getVisionAngle());
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTalonSlot(0);
        VisionModule.setLeds(false);
    }
}
