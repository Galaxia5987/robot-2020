package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shuffleboard.SmartDash;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

public class ShootAtVelocity extends CommandBase {
    static WebConstant targetVelocity = new WebConstant("shooterTargetVelocity", 80);
    double velocity;
    boolean manual = false;
    private final Shooter shooter;

    public ShootAtVelocity(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public ShootAtVelocity(Shooter shooter, double targetVelocity) {
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
        }else { setpoint = targetVelocity.get();
        }
        shooter.setSpeed(setpoint);
        FireLog.log("shooterSpeed", shooter.getSpeed());
        FireLog.log("shooterSetpoint", setpoint);
        SmartDash.putNumber("shooterVoltage", shooter.getMasterVoltage());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
