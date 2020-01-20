package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakePowerCell;
import frc.robot.subsystems.intake.commands.OuttakeBall;

public class LoadConveyor extends SequentialCommandGroup {

    public LoadConveyor(Intake intake, Conveyor conveyor, double speed, double timeout, int balls, int remainBalls){
        addRequirements(intake, conveyor);
        addCommands(
                new IntakePowerCell(speed, timeout),
                new ParallelCommandGroup(new OuttakeBall(speed, timeout), new FeedTurret(balls)),
                new FeedTurret(remainBalls)
        );
    }

}
