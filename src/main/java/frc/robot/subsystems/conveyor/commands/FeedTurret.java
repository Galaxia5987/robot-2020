package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Conveyor.FEED_TIMEOUT;
import static frc.robot.Constants.Conveyor.MAX_BALLS_AMOUNT;
import static frc.robot.RobotContainer.conveyor;

public class FeedTurret extends CommandBase {
    private int balls; //Shoot out X balls!
    private Timer timer = new Timer(); //Timer for the timeout of the turret, incase the ball count does not work
    private double timeout;

    public FeedTurret(int balls, double timeout) {
        addRequirements(conveyor);
        this.balls = MAX_BALLS_AMOUNT - balls;
        this.timeout = timeout;
    }


    public FeedTurret(int balls) {
        this(balls, FEED_TIMEOUT);
    }

    public FeedTurret() {
        this(Math.max(MAX_BALLS_AMOUNT,conveyor.getBallsCount())); //In case there are more than 5..
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
        return conveyor.getBallsCount() <= balls || timer.get() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        conveyor.stop();
    }
}