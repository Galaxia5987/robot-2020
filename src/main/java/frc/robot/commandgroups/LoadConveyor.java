package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakePowerCell;

import static frc.robot.Constants.Intake.INTAKE_POWER;

public class LoadConveyor extends ParallelCommandGroup {

    public LoadConveyor(Intake intake, Conveyor conveyor, double timeout){
        addRequirements(intake, conveyor);
        addCommands(
                // fold the intake down and intake balls
                new IntakePowerCell(intake, INTAKE_POWER).withTimeout(timeout),
                new FeedTurret(conveyor)
        );
    }

}