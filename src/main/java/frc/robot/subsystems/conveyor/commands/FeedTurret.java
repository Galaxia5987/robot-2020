package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Conveyor.MAX_BALLS_COUNT;
import static frc.robot.RobotContainer.conveyor;

public class FeedTurret extends CommandBase {
    private int remainBalls;
    private Timer timer = new Timer();

    public FeedTurret(int remainBalls) {
        addRequirements(conveyor);
        this.remainBalls = remainBalls;
    }

    public FeedTurret() {
        this(MAX_BALLS_COUNT);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        conveyor.feed();
    }

    @Override
    public void execute() {
        conveyor.feed();
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= remainBalls;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        conveyor.stop();
    }
}