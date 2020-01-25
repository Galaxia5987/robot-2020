package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_FEED_POWER;

/**
 * Open the mechanical stopper and feed Power Cells into the shooter.
 */
public class FeedTurret extends CommandBase {
    private Conveyor conveyor;

    public FeedTurret(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.openGate(true);
        conveyor.setPower(CONVEYOR_MOTOR_FEED_POWER);
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