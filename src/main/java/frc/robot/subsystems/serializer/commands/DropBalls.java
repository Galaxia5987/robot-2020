package frc.robot.subsystems.serializer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.serializer;

public class DropBalls extends CommandBase {
    private int balls;
    private Timer timer = new Timer();

    public DropBalls(int balls) {
        addRequirements(serializer);
        this.balls = balls;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}