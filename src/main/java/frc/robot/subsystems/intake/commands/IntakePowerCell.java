package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.intake;

public class IntakePowerCell extends CommandBase {
    private Timer timer = new Timer();
    private double speed;
    private double timeout;

    public IntakePowerCell(double speed, double timeout) {
        addRequirements(intake);
        this.speed = speed;
        this.timeout = timeout;
    }

    public IntakePowerCell(double speed) {
        this(speed, 0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intake.setPosition(Value.kReverse);
        intake.powerWheels(speed);
    }

    @Override
    public void execute() {
        intake.powerWheels(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.powerWheels(0);
    }
}
