package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class DeadbandProximity {
    private AnalogInput proximity;
    private double minDistance;
    private double maxDistance;

    public DeadbandProximity(int port, double minDistance, double maxDistance) {
        proximity = new AnalogInput(port);
        this.maxDistance = maxDistance;
        this.minDistance = minDistance;
    }

   
}
