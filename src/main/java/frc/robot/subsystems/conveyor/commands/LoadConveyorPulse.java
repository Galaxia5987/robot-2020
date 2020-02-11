package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.utilities.State;

import static frc.robot.Constants.Conveyor.*;

/**
 * Turn the conveyor in pulses and close the gate
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
        conveyor.setFunnelPower(FUNNEL_FEED_POWER);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() % (2 * PULSE_INTERVAL) <= PULSE_INTERVAL){
            conveyor.setConveyorPower(CONVEYOR_INTAKE_POWER);
        }
        else {
            conveyor.stopConveyor();
        }
    }

    @Override
    public boolean isFinished() {
        return conveyor.getBallsCount() >= 5;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stopAll();
    }
}