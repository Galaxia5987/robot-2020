package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class TurnLocalization extends CommandBase {
    private final Turret turret;

    public TurnLocalization(Turret turret) {
        this.turret = turret;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        turret.setAngle(turret.calculateTargetAngle());
    }

}
