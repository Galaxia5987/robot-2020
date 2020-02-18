package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import java.util.function.Supplier;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.Shooter.VELOCITY_TOLERANCE;

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
        conveyor.setGate(State.OPEN);
    }

    @Override
    public void execute() {
        if (smartFeed) {
        if (smartFeed && isShooting.get()) {
            if (isShooterReady.get() && isTurretReady.get()) {
                if (conveyor.isGateOpen()) {
                    conveyor.setConveyorPower(CONVEYOR_MOTOR_OPEN_FEED_POWER.get());
                    conveyor.setFunnelPower(FUNNEL_MOTOR_FEED_POWER.get());
                }
                else
                    conveyor.setGate(State.OPEN);

            }
            else {
                conveyor.setConveyorPower(0);
                conveyor.setFunnelPower(0);
                conveyor.setGate(State.CLOSE);
            }
        }
        else {
            conveyor.setConveyorPower(CONVEYOR_MOTOR_OPEN_FEED_POWER.get());
            conveyor.setFunnelPower(FUNNEL_MOTOR_FEED_POWER.get());
        }
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