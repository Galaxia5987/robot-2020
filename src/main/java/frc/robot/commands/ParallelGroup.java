package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;


public class ParallelGroup extends ParallelDeadlineGroup {
     public ParallelGroup(){
         super(new Parallel2());
         addCommands(
                 new Parallel1()
         );
     }

}
