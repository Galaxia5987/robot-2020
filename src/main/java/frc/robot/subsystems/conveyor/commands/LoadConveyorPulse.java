package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;

import static frc.robot.Constants.Conveyor.*;

/**
 * Turn the conveyor and close the gate
 */
public class LoadConveyorPulse extends CommandBase {
    private Conveyor conveyor;
    private Timer timer = new Timer();

    public LoadConveyorPulse(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setGate(State.CLOSE);
        timer.reset();
        conveyor.setConveyorPower(CONVEYOR_MOTOR_INTAKE_POWER.get());
        conveyor.setFunnelPower(FUNNEL_MOTOR_FEED_POWER.get());
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() > PULSE_INTERVAL.get()) {
            conveyor.stopAll();
        }
        else if(timer.get() > PULSE_INTERVAL.get() * 2) {
            conveyor.setConveyorPower(CONVEYOR_MOTOR_INTAKE_POWER.get());
            conveyor.setFunnelPower(FUNNEL_MOTOR_FEED_POWER.get());
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() >= 5;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}