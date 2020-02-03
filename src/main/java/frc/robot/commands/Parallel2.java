package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Test;


public class Parallel2 extends CommandBase {

    private Test test = new Test();
    private Timer timer = new Timer();

    public Parallel2() {
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        test.print("running2");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        test.print("STOP-AAAAAH");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= 3;
    }
}
