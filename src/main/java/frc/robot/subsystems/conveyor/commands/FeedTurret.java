package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
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
    private boolean smartFeed = false; //In cases where we want to feed at a constant rate. TODO: Deprecate this once we are confident with constant velocity checking.

    public FeedTurret(Conveyor conveyor) {
        this(conveyor, () -> true, () -> true);
        smartFeed = false;
    }

    public FeedTurret(Conveyor conveyor, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady){
        addRequirements(conveyor);
        this.conveyor = conveyor;
        this.isShooterReady = isShooterReady;
        this.isTurretReady = isTurretReady;
        smartFeed = true;
    }


    @Override
    public void initialize() {
        conveyor.openGate(true);
    }

    @Override
    public void execute() {
        if (smartFeed) {
            if (isShooterReady.get() && isTurretReady.get())
                conveyor.setPower(CONVEYOR_MOTOR_FEED_POWER);
            else
                conveyor.stop();
        }
        else
            conveyor.setConveyorPower(CONVEYOR_MOTOR_OPEN_FEED_POWER);
            conveyor.setFunnelPower(FUNNEL_MOTOR_FEED_POWER);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        conveyor.openGate(false);
    }
}