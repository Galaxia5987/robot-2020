package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.conveyor;

public class MinimizeConveyor extends CommandBase {

    public MinimizeConveyor() {
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.moveConveyor(-1);
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