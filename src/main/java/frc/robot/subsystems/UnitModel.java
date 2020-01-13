package frc.robot.subsystems;

public class UnitModel {
    final int ticksPerUnit;

    public UnitModel(int ticksPerUnit) {
        this.ticksPerUnit = ticksPerUnit;
    }

    public double toUnits(double ticks) {
        return ticks / ticksPerUnit;
    }

    public int toTicks(double units) {
        return (int) (ticksPerUnit * units);
    }

}
