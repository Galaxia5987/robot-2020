package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Test;


public class Parallel1 extends CommandBase {
    private Test test = new Test();

    public Parallel1() {
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        test.print("running1");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        test.print("PARALLEL_1_END");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
