package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.intake;

public class OuttakeBall extends CommandBase {
    private Timer timer = new Timer();
    private double speed;
    private double timeout;

    /**
     * this constructor is for a situation when you use an autonomous command.
     * please be aware that you should insert positive {@param speed)'s number.
     *
     * @param speed
     * @param timeout
     */
    public OuttakeBall(double speed, double timeout) {
        addRequirements(intake);
        this.speed = -speed;
        this.timeout = timeout;
    }

    public OuttakeBall(double speed) {
        this(speed, 0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intake.setPosition(false);
        intake.powerWheels(speed);
    }

    @Override
    public void execute() {
        intake.powerWheels(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= timeout || timeout == 0;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.powerWheels(0);
    }
}
