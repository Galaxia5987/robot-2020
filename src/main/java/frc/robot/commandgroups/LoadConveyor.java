package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.Gate;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakePowerCell;
import frc.robot.subsystems.intake.commands.ToggleIntake;

public class LoadConveyor extends SequentialCommandGroup {

    public LoadConveyor(Intake intake, Conveyor conveyor, boolean down, boolean gate, double speed, double timeout){
        addRequirements(intake, conveyor);
        addCommands(
                new ToggleIntake(down),
                // move the power cells to the turret and
                new ParallelCommandGroup(new IntakePowerCell(speed, timeout), new FeedTurret(), new Gate(conveyor, gate))
        );
    }

}