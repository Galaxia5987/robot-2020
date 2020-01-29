package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;

/**
 * Changes the current reading of the amount of balls in the robot. can be used both for incrementing and decrementing.
 */
public class ChangeBallCount extends InstantCommand {
    Conveyor conveyor;
    int amount;
    public ChangeBallCount(Conveyor conveyor, int amount){
        this.conveyor = conveyor;
        this.amount = amount;
    }

    public void initialize(){
        conveyor.incrementBallsCount(amount);
    }
}
