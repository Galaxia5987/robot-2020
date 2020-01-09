package frc.robot.subsystems;

public class UnitModel {
    int ticksPerUnit;

    public UnitModel(int ticksPerUnit) {
        this.ticksPerUnit = ticksPerUnit;
    }

    public double convertToUnits(double ticks) {
        return ticks / ticksPerUnit;
    }

    public int convertToTicks(double units) {
        return (int) (ticksPerUnit * units);
    }

}
