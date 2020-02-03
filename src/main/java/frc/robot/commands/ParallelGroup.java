package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Test;

public class ParallelGroup extends ParallelDeadlineGroup {
     public ParallelGroup(Test test){
         super(new Parallel2());
         addCommands(
                 new Parallel1()
         );
     }

}
