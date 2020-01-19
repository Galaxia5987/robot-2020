package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.conveyor;
import static frc.robot.Constants.Conveyor.*;

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
        return conveyor.feederSensedObject();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}