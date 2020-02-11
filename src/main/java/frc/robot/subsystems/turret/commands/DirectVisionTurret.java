package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

import static frc.robot.Constants.Turret.*;

public class DirectVisionTurret extends CommandBase {
    private Turret turret;
    private PIDController anglePid = new PIDController(DIRECT_VISION_KP.get(), DIRECT_VISION_KI.get(), DIRECT_VISION_KD.get());

    public DirectVisionTurret(Turret turret) {
        addRequirements(turret);
        this.turret = turret;
    }

    @Override
    public void initialize() {
        VisionModule.setLeds(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        anglePid.setP(DIRECT_VISION_KP.get());
        anglePid.setI(DIRECT_VISION_KI.get());
        anglePid.setD(DIRECT_VISION_KD.get());
        double power = -anglePid.calculate(VisionModule.getVisionAngle(), 0);
        power = power + Math.copySign(FRICTION_COEFFICIENT.get(), power);
        turret.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLeds(false);
    }
}
