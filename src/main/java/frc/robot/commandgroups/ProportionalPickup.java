package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.LoadConveyor;
import frc.robot.subsystems.conveyor.commands.LoadConveyorPulse;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.intake.commands.ProportionalIntake;

/**
 * Command group which turns the intake and the conveyor at the same time.
 */
public class ProportionalPickup extends ParallelDeadlineGroup {

    public ProportionalPickup(Intake intake, Conveyor conveyor, Drivetrain drivetrain) {
        super(new LoadConveyor(conveyor));
        addCommands(
                new ProportionalIntake(intake, drivetrain)
        );
    }
}