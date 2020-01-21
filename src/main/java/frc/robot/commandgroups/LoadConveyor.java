package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.MoveGate;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakePowerCell;
import frc.robot.subsystems.intake.commands.ToggleIntake;

import static frc.robot.Constants.Conveyor.CLOSE_GATE;
import static frc.robot.Constants.Intake.INTAKE_SPEED;

public class LoadConveyor extends SequentialCommandGroup {

    public LoadConveyor(Intake intake, Conveyor conveyor, double timeout){
        addRequirements(intake, conveyor);
        addCommands(
                // fold the intake out
                new ToggleIntake(),
                // move the power cells towards the turret and close the gate
                new ParallelCommandGroup(new IntakePowerCell(INTAKE_SPEED, timeout), new FeedTurret(), new MoveGate(conveyor, CLOSE_GATE))
        );
    }

}