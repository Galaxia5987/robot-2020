package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

import static frc.robot.Constants.Turret.*;

public class DirectVisionTurret extends CommandBase {
    private Turret turret;
    private PIDController anglePid = new PIDController(DIRECT_VISION_KP, DIRECT_VISION_KI, DIRECT_VISION_KD);

    public DirectVisionTurret(Turret turret) {
        addRequirements(turret);
        this.turret = turret;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        anglePid.setP(DIRECT_VISION_KP);
        anglePid.setI(DIRECT_VISION_KI);
        anglePid.setD(DIRECT_VISION_KD);
        double power = -anglePid.calculate(VisionModule.getVisionAngle(), 0);
        if(VisionModule.targetSeen())
            turret.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
        turret.setAngle(turret.getAngle());
    }
}
