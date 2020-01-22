package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climber;

public class ClimbAndBalance extends SequentialCommandGroup {

    public ClimbAndBalance(Climber climber , double setpoint){
        addRequirements(climber);
        addCommands(
                new RiseToHeight(climber, setpoint),
                new BalanceRobot(climber, setpoint)

        );
    }

}
