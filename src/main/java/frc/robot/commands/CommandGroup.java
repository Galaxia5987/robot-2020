package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandGroup extends SequentialCommandGroup {

    public CommandGroup(double timeout) {
        addCommands(
               new PrintCommand("running").withTimeout(timeout),
               new WithTimeoutTest().withInterrupt(() -> true),
               new PrintCommand("STOPPED")
        );
    }

}
