package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.MASTER_INVERTED;
import static frc.robot.Ports.Intake.*;

/**
 * @author Barel
 * @version 1.0.0
 * <p>
 * this class defining methods to the intake subsystem to apply in the commands.
 * {@using VictorSPX}
 * {@using DoubleSolenoid}
 */
public class Intake extends SubsystemBase {
    private VictorSPX masterMotor = new VictorSPX(MASTER);
    private DoubleSolenoid solenoid = new DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, SOLENOID_BACKWARD_CHANNEL);

    public Intake() {
        masterMotor.setInverted(MASTER_INVERTED);
    }

    /**
     * get the current position of the solenoid
     *
     * @return the position of the solenoid as a Value class instance
     */
    public Value getPosition() {
        return solenoid.get();
    }

    /**
     * set the new position of the solenoid.
     * can be either Forward (Up) or Reverse (Down).
     *
     * @param direction the desired direction for the solenoid
     */
    public void setPosition(Value direction) {
        solenoid.set(direction);
    }

    /**
     * Toggle the position of the intake.
     * if you want to choose manually use {@see setPosition(Value)} instead.
     */
    public void togglePosition() {
        if (getPosition() == Value.kForward) {
            setPosition(Value.kReverse);
        } else if (getPosition() == Value.kReverse)
            setPosition(Value.kForward);
    }

    /**
     * apply power on the wheel to spin them based on the speed you insert.
     *
     * @param speed the speed to apply on the intake's wheels (in percents)
     */
    public void applyPowerOnWheels(double speed) {
        masterMotor.set(ControlMode.PercentOutput, speed);
    }
}
