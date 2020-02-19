package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.LoadConveyorPulse;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;

/**
 * Command group which turns the intake and the conveyor at the same time.
 */
public class PickupBalls extends ParallelDeadlineGroup {

    public PickupBalls(Intake intake, Conveyor conveyor) {
        super(new LoadConveyorPulse(conveyor));
        addCommands(
                new IntakeBalls(intake)
        );
    }
}