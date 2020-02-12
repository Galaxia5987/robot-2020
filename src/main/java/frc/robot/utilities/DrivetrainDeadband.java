package frc.robot.utilities;

public class DrivetrainDeadband {
    private double minValue;
    private double current;

    public DrivetrainDeadband(double current, double minValue){
        this.current = current;
    }

    public void update(){}
    
}
