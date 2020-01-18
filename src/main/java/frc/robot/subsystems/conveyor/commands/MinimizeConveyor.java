package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.conveyor;

public class MinimizeConveyor extends CommandBase {

    public MinimizeConveyor() {
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.minimizeConveyor();
    }

    @Override
    public void execute() {
        conveyor.minimizeConveyor();
    }

    @Override
    public boolean isFinished() {
        return conveyor.entrySensedObject();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}