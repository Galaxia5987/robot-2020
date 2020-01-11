package frc.robot.subsystems.serializer.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Serializer.BALL_DROP_TIME;
import static frc.robot.RobotContainer.serializer;

public class DropBalls extends CommandBase {
    private int balls;
    private Timer timer = new Timer();

    public DropBalls(int balls) {
        addRequirements(serializer);
        this.balls = balls;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        serializer.drop();
    }

    @Override
    public void execute() {
        serializer.drop();
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