package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.turret.Turret;

public class ResetDelta extends InstantCommand {
    private final Turret turret;

    public ResetDelta(Turret turret) {
        this.turret = turret;
    }

    @Override
    public void initialize() {
        this.turret.resetOffset();
    }

}
