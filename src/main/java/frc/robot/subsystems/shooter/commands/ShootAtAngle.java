package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.valuetuner.WebConstant;

public class ShootAtAngle extends InstantCommand {
    static WebConstant targetAngle = new WebConstant("shooterTargetAngle", 0);
    double velocity;
    boolean manual = false;
    private final Shooter shooter;

    public ShootAtAngle(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public ShootAtAngle(Shooter shooter, double targetVelocity) {
        this.shooter = shooter;
        this.velocity = targetVelocity;
        manual =true;
        addRequirements(shooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double setpoint;
        if (manual){
             setpoint = velocity;
        }else { setpoint = targetAngle.get();
        }
        shooter.setHoodAngle(setpoint);

    }

    // Make this return true when this Command no longer needs to run execute()

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }

}
