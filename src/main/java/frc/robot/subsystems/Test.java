package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {

    public Test(){

    }

    public void print(String print){
        System.out.println(print);
    }

    public void stop() {
        System.out.println("Stopped");
    }
}