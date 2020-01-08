package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Ports.Intake.*;

public class Intake extends SubsystemBase {
    private VictorSPX masterMotor = new VictorSPX(MASTER);
    private Solenoid solenoid = new Solenoid(SOLENOID);

    public Intake() {
        masterMotor.setInverted(MASTER_INVERTED);
    }

    public void setPosition(Direction direction) {
        if (direction == Direction.UP)
            solenoid.set(true);
        else
            solenoid.set(false);
    }

    public void togglePosition() {
        if (getPosition() == Direction.UP) {
            setPosition(Direction.DOWN);
            return;
        }
        setPosition(Direction.UP);
    }

    public Direction getPosition() {
        if (solenoid.get())
            return Direction.UP;
        return Direction.DOWN;
    }

    public void applyPowerOnWheels(double speed) {
        masterMotor.set(ControlMode.PercentOutput, speed);
    }

    public enum Direction {
        UP, DOWN
    }
}
