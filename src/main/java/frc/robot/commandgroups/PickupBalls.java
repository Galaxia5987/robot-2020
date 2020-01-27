package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.LoadConveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;

import static frc.robot.Constants.Intake.INTAKE_POWER;

public class PickupBalls extends ParallelCommandGroup {

    public PickupBalls(Intake intake, Conveyor conveyor, double timeout){
        addRequirements(intake, conveyor);
        addCommands(
                // fold the intake down and intake balls
                new IntakeBalls(intake, INTAKE_POWER).withTimeout(timeout),
                new LoadConveyor(conveyor)
        );
    }

}