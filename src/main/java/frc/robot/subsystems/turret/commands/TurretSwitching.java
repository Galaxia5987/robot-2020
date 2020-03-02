package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.FullLocalization;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class TurretSwitching extends CommandBase {
    private final LocalizationTurret localizationTurret;
    private final VisionTurret visionTurret;
    private final Turret turret;
    private final Timer timer = new Timer();
    private static final double VISION_TIMEOUT = 0.3;

    public TurretSwitching(Turret turret, FullLocalization localization, Drivetrain drivetrain) {
        this.turret = turret;
        this.localizationTurret = new LocalizationTurret(turret, drivetrain);
        this.visionTurret = new VisionTurret(turret);
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (VisionModule.targetSeen()) {
            visionTurret.execute();
            timer.stop();
            timer.reset();
        } else if (turret.inDeadZone()) {
            localizationTurret.execute();
        }
        else {
            if(timer.get() == 0)
                timer.start();
        }
        if (timer.get() > VISION_TIMEOUT) {
            localizationTurret.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
