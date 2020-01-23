package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Conveyor.CONVEYOR_MOTOR_RETURN_VELOCITY;
import static frc.robot.RobotContainer.conveyor;

/**
 * Minimize conveyor back
 */
public class MinimizeConveyor extends CommandBase {

    public MinimizeConveyor() {
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
        return conveyor.intakeSensedObject();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}