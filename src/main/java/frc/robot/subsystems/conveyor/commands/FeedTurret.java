package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_FEED_VELOCITY;

public class FeedTurret extends CommandBase {
    private Conveyor conveyor;
    private boolean open;

    public FeedTurret(Conveyor conveyor, boolean open) {
        this.conveyor = conveyor;
        this.open = open;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.openGate(open);
        conveyor.setConveyorSpeed(CONVEYOR_MOTOR_FEED_VELOCITY);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}