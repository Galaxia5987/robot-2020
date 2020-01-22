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
        anglePid.setSetpoint(VISION_SETPOINT);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        anglePid.setSetpoint(VISION_SETPOINT);
        anglePid.setPID(VISION_KP, VISION_KI, VISION_KD);
        turret.setAngle(anglePid.getSetpoint());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(VISION_SETPOINT - turret.getVisionAngle()) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
