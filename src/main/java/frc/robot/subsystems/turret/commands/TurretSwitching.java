package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class TurretSwitching extends CommandBase {
    private final TurnLocalization turnLocalization;
    private final VisionTurret visionTurret;
    private final Turret turret;
    private final Timer timer = new Timer();

    public TurretSwitching(Turret turret) {
        this.turret = turret;
        this.turnLocalization = new TurnLocalization(turret);
        this.visionTurret = new VisionTurret(turret);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if(turret.hasVisionAngle() && turret.inCorrectRange()) {
            timer.reset();
            visionTurret.execute();
        }
        else {
            turret.setAngle(turret.getAngle());
            timer.start();
            if(timer.get() > Constants.Turret.VISION_TIMEOUT_SECONDS) {
                turnLocalization.execute();
            }
        }
    }
}
