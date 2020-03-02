package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.valuetuner.WebConstant;

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
    private Timer timer = new Timer();

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
        conveyor.setConveyorPower(-FEED_OUTTAKE_POWER.get());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() < OUTTAKE_TIME) return;

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
            conveyor.setGate(State.OPEN);
            conveyor.setConveyorPower(CONVEYOR_FEED_POWER);
            conveyor.setFunnelPower(FUNNEL_INTAKE_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }
}