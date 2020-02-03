package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Test;

public class CommandGroup extends SequentialCommandGroup {

    public CommandGroup(double timeout) {
        addCommands(
               new WithTimeoutTest().withTimeout(timeout),
               new PrintCommand("STOPPED")
        );
    }

}