package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

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
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        anglePid.setP(DIRECT_VISION_KP.get());
        anglePid.setI(DIRECT_VISION_KI.get());
        anglePid.setD(DIRECT_VISION_KD.get());
        turret.setPower(-anglePid.calculate(turret.getVisionAngle(), 0));
    }

}
