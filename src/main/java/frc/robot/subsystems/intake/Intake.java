package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.State;

import java.util.function.Supplier;

import static frc.robot.Constants.TALON_TIMEOUT;
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
    private TalonSRX motor = new TalonSRX(MOTOR);
    private DoubleSolenoid retractor = new DoubleSolenoid(FOLD_SOLENOID_FORWARD, FOLD_SOLENOID_REVERSE);
    private Supplier<Integer> proximitySupplier = motor::getSelectedSensorPosition;

    public Intake() {
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, TALON_TIMEOUT);
        motor.setInverted(MOTOR_INVERTED);
    }

    /**
     * get the current position of the intake.
     *
     * @return whether the intake mechanism is open to intake Power Cells.
     */
    public boolean isOpen() {
        return retractor.get() == Value.kForward;
    }

    /**
     * set the position of the retractor piston holding the intake.
     * can be either
     *
     * @param open whether the Power Cell intake mechanism will be open.
     *           If set to false, the intake will close inside the robot.
     */
    public void setPosition(boolean open) {
        retractor.set(open == IS_FORWARD_OPEN ? Value.kForward : Value.kReverse);
    }

    /**
     * Toggle the position of the intake.
     * if you wish to change the position manually, use {@link #setPosition(boolean)} instead.
     */
    public void togglePosition() {
        setPosition(!isOpen());
    }

    public Supplier<Integer> getProximitySupplier(){
        return proximitySupplier;
    }
    /**
     * OPEN is in the state where the intake is functional
     * CLOSE for the state of bringing the intake in.
     *
     * @param state state of the intake, OPEN / CLOSE / TOGGLE
     */
    public void setPosition(State state){
        switch (state) {
            case OPEN:
                retractor.set(Value.kForward);
                break;
            case CLOSE:
                retractor.set(Value.kReverse);
                break;
            case TOGGLE:
                if (isOpen())
                    retractor.set(Value.kForward);
                else
                    retractor.set(Value.kReverse);
                break;
        }
    }
    /**
     * apply power on the wheel to spin them based on the power you insert.
     *
     * @param power the power to apply on the intake's wheels (in percents)
     */
    public void powerWheels(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
}
