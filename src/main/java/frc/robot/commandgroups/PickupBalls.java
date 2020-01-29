package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.LoadConveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;

import static frc.robot.Constants.Intake.INTAKE_POWER;

/**
 * Command group which turns the intake and the conveyor at the same time.
 */
public class PickupBalls extends ParallelDeadlineGroup {

    public PickupBalls(Intake intake, Conveyor conveyor){
        super(new LoadConveyor(conveyor));
        addRequirements(intake, conveyor);
        addCommands(
                new IntakeBalls(intake, INTAKE_POWER)
        );
    }
}