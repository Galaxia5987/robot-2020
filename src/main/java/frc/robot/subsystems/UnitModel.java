package frc.robot.subsystems;

public class UnitModel {
    int ticksPerUnit;

    public UnitModel(int ticksPerUnit) {
        this.ticksPerUnit = ticksPerUnit;
    }

    public double toUnits(double ticks) {
        return ticks / ticksPerUnit;
    }

    public int toTicks(double units) {
        return (int) (ticksPerUnit * units);
    }

    public double toVelocity(double ticks100ms){
        return (ticks100ms / ticksPerUnit) * 10;
    }

    public int toTicks100ms(double velocity){
        return (int) (velocity * ticksPerUnit / 10);
    }

}
