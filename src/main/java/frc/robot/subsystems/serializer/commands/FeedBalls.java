package frc.robot.subsystems.serializer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.serializer;

public class FeedBalls extends CommandBase {
    public int ballsCount;

    public FeedBalls(int ballsCount) {
        addRequirements(serializer);
        this.ballsCount = ballsCount;
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