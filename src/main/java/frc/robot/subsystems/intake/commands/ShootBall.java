package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import static frc.robot.Robot.intake;

public class ShootBall extends CommandBase {
    private Timer timer = new Timer();
    private double speed;
    private double timeout;

    public ShootBall(double speed, double timeout) {
        addRequirements(intake);
        this.speed = -speed;
        this.timeout = timeout;
    }

    public ShootBall(double speed) {
        this(speed, 0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intake.setPosition(Intake.Direction.DOWN);
        intake.applyPowerOnWheels(speed);
    }

    @Override
    public void execute() {
        intake.applyPowerOnWheels(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.applyPowerOnWheels(0);
    }
}
