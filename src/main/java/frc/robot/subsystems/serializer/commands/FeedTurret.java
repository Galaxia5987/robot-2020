package frc.robot.subsystems.serializer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Serializer.BALL_FEED_TIME;
import static frc.robot.RobotContainer.serializer;

public class FeedTurret extends CommandBase {
    public int ballsCount;
    private Timer timer = new Timer();

    public FeedTurret(int ballsCount) {
        addRequirements(serializer);
        this.ballsCount = ballsCount;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        serializer.feed();
    }

    @Override
    public void execute() {
        serializer.feed();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= (ballsCount * BALL_FEED_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        serializer.stop();
    }
}