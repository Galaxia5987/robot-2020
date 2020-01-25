package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class TurretSwitchingCommand extends CommandBase {
    private final JoystickTurret joystickTurret;
    private final TurnLocalization turnLocalization;
    private final VisionTurret visionTurret;
    private final Turret turret;

    public TurretSwitchingCommand(Turret turret) {
        this.turret = turret;
        this.joystickTurret = new JoystickTurret(turret);
        this.turnLocalization = new TurnLocalization(turret);
        this.visionTurret = new VisionTurret(turret);
    }

    @Override
    public void execute() {
        if(turret.hasVisionAngle() && turret.inCorrectRange()) {
            visionTurret.execute();
        }
        else {
            turnLocalization.execute();
        }
    }
}
