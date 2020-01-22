package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class VisionTurret extends CommandBase {
    private Turret turret;
//    private MiniPID anglePid = new MiniPID(VISION_KP, VISION_KI, VISION_KD);

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
//        anglePid.setPID(VISION_KP, VISION_KI, VISION_KD);
//        turret.setAngle(anglePid.getOutput(turret.getVisionAngle(), 0));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
//        return Math.abs(turret.getVisionAngle()) <= VISION_ANGLE_THRESHOLD;
        return false;
    }
}
