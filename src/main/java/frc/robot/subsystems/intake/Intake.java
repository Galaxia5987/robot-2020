package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.State;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Ports.Intake.*;

/**
 * @author Barel
 * @version 1.0.0
 * <p>
 * this class defining methods to the intake subsystem to apply in the commands.
 * {@using 2x VictorSPX}
 * {@using DoubleSolenoid}
 */
public class Intake extends SubsystemBase {
    private VictorSPX masterMotor = new VictorSPX(MASTER);
    private DoubleSolenoid redactor = new DoubleSolenoid(FOLD_SOLENOID_FORWARD, FOLD_SOLENOID_REVERSE);

    public Intake() {
        masterMotor.setInverted(MASTER_INVERTED);
    }

    /**
     * get the current position of the redactor
     *
     * @return the position of the redactor as a Value class instance
     */
    public boolean isFolded() {
        return redactor.get() == Value.kForward;
    }

    /**
     * set the position of the redactor.
     * can be either Forward (Folded) or Reverse (Unfolded).
     *
     * @param up whether the redactor should move up.
     *           Note that in case inserting false, the redactor will move down.
     */
    public void setPosition(boolean up) {
        redactor.set(up ? Value.kForward : Value.kReverse);
    }

    /**
     * Toggle the position of the intake.
     * if you wish to change the position manually, use {@link #setPosition(boolean)} instead.
     */
    public void togglePosition() {
        setPosition(!isFolded());
    }

    /**
     * OPEN is in the state where the intake is functional
     * CLOSE for the state of bringing the intake in.
     * Toggle switch the state to the opposite state,
     * for example if the state is OPEN he would become CLOSE.
     *
     * @param state state of the intake, OPEN / CLOSE / TOGGLE
     */
    public void setPosition(State state){
        switch (state) {
            case OPEN:
                redactor.set(Value.kForward);
                break;
            case CLOSE:
                redactor.set(Value.kReverse);
                break;
            case TOGGLE:
                if (isFolded())
                    redactor.set(Value.kForward);
                else
                    redactor.set(Value.kReverse);
                break;
        }
    }
    /**
     * apply power on the wheel to spin them based on the power you insert.
     *
     * @param power the power to apply on the intake's wheels (in percents)
     */
    public void powerWheels(double power) {
        masterMotor.set(ControlMode.PercentOutput, power);
    }
}
