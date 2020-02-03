package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Test;

public class WhileHeldTest extends CommandBase {
    private Test test = new Test();
    public WhileHeldTest(){

    }

    @Override
    public void execute(){
        test.print("holding");
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        test.stop();
    }

}
