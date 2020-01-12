package frc.robot.subsystems.serializer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Serializer.BALL_DROP_TIME;
import static frc.robot.Constants.Serializer.MAX_BALLS_COUNT;
import static frc.robot.RobotContainer.serializer;

public class MoveBallsToEntry extends CommandBase {
    private int balls;
    private Timer timer = new Timer();

    public MoveBallsToEntry(int balls) {
        addRequirements(serializer);
        this.balls = balls;
    }

    public MoveBallsToEntry() {
        this(MAX_BALLS_COUNT);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        serializer.minimizeConveyor();
    }

    @Override
    public void execute() {
        serializer.minimizeConveyor();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > (balls * BALL_DROP_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        serializer.stop();
    }
}