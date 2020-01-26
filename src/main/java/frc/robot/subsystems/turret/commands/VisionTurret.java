package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;

public class VisionTurret extends CommandBase {
    private Turret turret;
    private PIDController anglePid = new PIDController(VISION_KP, VISION_KI, VISION_KD);

    public VisionTurret(Turret turret) {
        addRequirements(turret);
        this.turret = turret;
    }

    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (turret.inCorrectRange(turret.getAngle() + turret.getVisionAngle() + turret.getDelta())) {
            turret.setPower(anglePid.calculate(turret.getVisionAngle() + turret.getDelta(), 0));
        } else {
            turret.setPower(0); //TODO: handle the situation where the target is out of the turret's boundaries.
        }

    }

}
