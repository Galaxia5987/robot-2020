package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class PullBall extends CommandBase {
    private Timer timer = new Timer();
    private double speed;
    private double timeout;

    public PullBall(double speed, double timeout) {
        addRequirements(Robot.intake);
        this.speed = speed;
        this.timeout = timeout;
    }

    public PullBall(double speed) {
        this(speed, 0); 
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
