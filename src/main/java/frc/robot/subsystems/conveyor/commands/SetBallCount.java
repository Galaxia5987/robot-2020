package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;

/**
 * Set the ball count to a specific number.
 * This command was created for simplicity when reading the code
 */
public class SetBallCount extends InstantCommand {
    Conveyor conveyor;
    int amount;
    public SetBallCount(Conveyor conveyor, int setAmount){
        this.conveyor = conveyor;
        this.amount = setAmount;
    }

    public void initialize(){
        conveyor.setBallsCount(amount);
    }
}
