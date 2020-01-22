package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Conveyor.MAX_BALLS_AMOUNT;
import static frc.robot.RobotContainer.conveyor;

public class FeedTurret extends CommandBase {
    private int balls; //Shoot out X balls!

    public FeedTurret(int balls) {
        addRequirements(conveyor);
        this.balls = MAX_BALLS_AMOUNT - balls;
    }

    public FeedTurret() {
        this(Math.max(MAX_BALLS_AMOUNT, conveyor.getBallsCount())); //In case there are more than 5..
    }

    @Override
    public void initialize() {
        conveyor.feed();
    }

    @Override
    public void execute() {
        conveyor.feed();
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= balls;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}