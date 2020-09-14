package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
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
    private final boolean outtake;
    private Conveyor conveyor;
    private Intake intake;
    private Supplier<Boolean> isShooterReady;
    private Supplier<Boolean> isTurretReady;
    private Supplier<Boolean> isShooting;
    private boolean smartFeed = false; //In cases where we want to feed at a constant rate. TODO: Deprecate this once we are confident with constant velocity checking.
    private Timer timer = new Timer();

    public FeedTurret(Conveyor conveyor) {
        this(conveyor, () -> true, () -> true, () -> true);
        smartFeed = false;
    }

    public FeedTurret(Conveyor conveyor, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady, Supplier<Boolean> isShooting) {
        this(conveyor, null, isShooterReady, isTurretReady, isShooting, true);
    }

    public FeedTurret(Conveyor conveyor, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady, Supplier<Boolean> isShooting, boolean outtake) {
        this(conveyor, null, isShooterReady, isTurretReady, isShooting, outtake);
    }

    public FeedTurret(Conveyor conveyor, Intake intake, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady, Supplier<Boolean> isShooting) {
        this(conveyor, intake, isShooterReady, isTurretReady, isShooting, true);
    }

    public FeedTurret(Conveyor conveyor, Intake intake, Supplier<Boolean> isShooterReady, Supplier<Boolean> isTurretReady, Supplier<Boolean> isShooting, boolean outtake) {
        addRequirements(conveyor);
        if(intake != null)
            addRequirements(intake);
        this.conveyor = conveyor;
        this.intake = intake; //if the intake parameter was null, it wont be used in the periodic
        this.isShooterReady = isShooterReady;
        this.isTurretReady = isTurretReady;
        this.isShooting = isShooting;
        smartFeed = true;
        this.outtake = outtake;
    }

    @Override
    public void initialize() {
        conveyor.setGate(State.OPEN);
        if (outtake)
            conveyor.setConveyorPower(-FEED_OUTTAKE_POWER.get());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (outtake && timer.get() < OUTTAKE_TIME) return;

        if (smartFeed && isShooting.get()) {
            if (isShooterReady.get() && isTurretReady.get()) {
                conveyor.feed();
                if (intake != null)
                    intake.powerWheels(Constants.Intake.FEED_POWER);
            } else {
                conveyor.setConveyorPower(0);
                conveyor.setFunnelPower(0);
            }
        } else if (!smartFeed) {
            if (isShooting.get())
                conveyor.setGate(State.OPEN);
            else
                conveyor.setGate(State.CLOSE);

            conveyor.setConveyorPower(CONVEYOR_FEED_POWER);
            conveyor.setFunnelPower(FUNNEL_INTAKE_POWER);
            if (intake != null)
                intake.powerWheels(Constants.Intake.FEED_POWER);

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
        if(intake != null)
            intake.powerWheels(0);
    }
}