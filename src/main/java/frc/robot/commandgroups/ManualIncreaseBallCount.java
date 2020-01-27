package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakePowerCell;

import static frc.robot.Constants.Intake.INTAKE_POWER;

public class ManualIncreaseBallCount extends ParallelCommandGroup {

    public ManualIncreaseBallCount(Intake intake, Conveyor conveyor, double intakeTimeout, double feedTimeout){
        addRequirements(intake, conveyor);
        addCommands(
                new IntakePowerCell(intake, INTAKE_POWER).withTimeout(intakeTimeout),
                new FeedTurret(conveyor).withTimeout(feedTimeout)
        );
    }
}
