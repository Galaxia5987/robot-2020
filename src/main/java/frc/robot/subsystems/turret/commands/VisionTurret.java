package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

import static frc.robot.Constants.Turret.ANGLE_THRESHOLD;

public class VisionTurret extends CommandBase {
    private final Turret turret;
    private final boolean end;

    public VisionTurret(Turret turret) {
        this(turret, false);
    }

    public VisionTurret(Turret turret, boolean end) {
        this.turret = turret;
        this.end = end;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    @Override
    public void execute() {
        if(turret.inDeadZone() || !VisionModule.targetSeen()) return;
        turret.setAngle(turret.getAngle() + VisionModule.getVisionAngle());
    }

    @Override
    public boolean isFinished() {
        return end && turret.isTurretReady();
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
