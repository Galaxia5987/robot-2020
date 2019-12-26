package robot.subsystems.examplesubsystem.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class ExampleCommand extends Command {

    private double speed;

    public ExampleCommand(double speed) {
        requires(Robot.m_example);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        System.out.println(speed);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {

    }

    @Override
    protected void end() {
    }
}
