package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.INTAKE_MOTOR_INVERTED;
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
    private VictorSPX intakeMotor = new VictorSPX(INTAKE);
    private DoubleSolenoid redactor = new DoubleSolenoid(FOLD_SOLENOID_FORWARD, FOLD_SOLENOID_REVERSE);

    public Intake() {
        intakeMotor.setInverted(INTAKE_MOTOR_INVERTED);
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
     * apply power on the wheel to spin them based on the speed you insert.
     *
     * @param speed the speed to apply on the intake's wheels (in percents)
     */
    public void powerWheels(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
}
