package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_RETURN_VELOCITY;

/**
 * Minimize conveyor back
 */
public class MinimizeConveyor extends CommandBase {

    private final Conveyor conveyor;

    public MinimizeConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setConveyorSpeed(CONVEYOR_MOTOR_RETURN_VELOCITY);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return conveyor.intakeSensedBall();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}