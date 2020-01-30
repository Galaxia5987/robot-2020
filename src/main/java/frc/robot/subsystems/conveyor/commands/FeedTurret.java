package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;

import java.util.function.Supplier;

import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_FEED_POWER;
import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_OPEN_FEED_POWER;

/**
 * Open the mechanical stopper and feed Power Cells into the shooter.
 */
public class FeedTurret extends CommandBase {
    private Conveyor conveyor;
    private Supplier<Boolean> isShooterReady;
    private Supplier<Boolean> isTurretReady;
    private boolean smartFeed = false; //In cases where we want to feed at a constant rate. TODO: Deprecate this once we are confident with constant velocity checking.

    public FeedTurret(Conveyor conveyor) {
        this(conveyor, () -> true, () -> true);
        smartFeed = false;
    }

    public FeedTurret(Conveyor conveyor, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady) {
        addRequirements(conveyor);
        this.conveyor = conveyor;
        this.isShooterReady = isShooterReady;
        this.isTurretReady = isTurretReady;
        smartFeed = true;
    }


    @Override
    public void initialize() {
        conveyor.setGate(State.OPEN);
    }

    @Override
    public void execute() {
        if (smartFeed) {
            if (isShooterReady.get() && isTurretReady.get())
                conveyor.setPower(CONVEYOR_MOTOR_FEED_POWER);
            else
                conveyor.stop();
        } else
            conveyor.setPower(CONVEYOR_MOTOR_OPEN_FEED_POWER);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        conveyor.setGate(State.CLOSE);
    }
}