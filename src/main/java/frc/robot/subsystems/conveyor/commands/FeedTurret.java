package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.Supplier;

import static frc.robot.Constants.Conveyor.*;

/**
 * Open the mechanical stopper and feed Power Cells into the shooter.
 */
public class FeedTurret extends CommandBase {
    private Conveyor conveyor;
    private Supplier<Boolean> isShooterReady;
    private Supplier<Boolean> isTurretReady;
    private Supplier<Boolean> isShooting;
    private boolean smartFeed = false; //In cases where we want to feed at a constant rate. TODO: Deprecate this once we are confident with constant velocity checking.

    public FeedTurret(Conveyor conveyor) {
        this(conveyor, () -> true, () -> true, () -> true);
        smartFeed = false;
    }

    public FeedTurret(Conveyor conveyor, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady, Supplier<Boolean> isShooting){
        addRequirements(conveyor);
        this.conveyor = conveyor;
        this.isShooterReady = isShooterReady;
        this.isTurretReady = isTurretReady;
        this.isShooting = isShooting;
        smartFeed = true;
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (smartFeed && isShooting.get()) {
            if (isShooterReady.get() && isTurretReady.get()) {
                conveyor.feed();
            }
            else {
                conveyor.setConveyorPower(0);
                conveyor.setFunnelPower(0);
            }
        }
        else if(!smartFeed) {
            conveyor.setConveyorPower(CONVEYOR_FEED_POWER);
            conveyor.setFunnelPower(FUNNEL_INTAKE_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }
}