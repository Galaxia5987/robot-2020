package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

public class ShootAtVelocity extends CommandBase {
    private final Shooter shooter;
    static WebConstant targetVelocity = new WebConstant("shooterTargetVelocity", 0);
    public ShootAtVelocity(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double setpoint = targetVelocity.get();
        shooter.setSpeed(setpoint);
        FireLog.log("shooterSpeed", shooter.getSpeed());
        FireLog.log("shooterSetpoint", setpoint);
        SmartDashboard.putNumber("shooterVoltage", shooter.footFungus());
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
