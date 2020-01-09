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
    private VictorSPX intakeMotor = new VictorSPX(MASTER);
    private DoubleSolenoid retracter = new DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, SOLENOID_BACKWARD_CHANNEL);

    public Intake() {
        intakeMotor.setInverted(MASTER_INVERTED);
    }

    /**
     * get the current position of the retracter
     *
     * @return the position of the retracter as a Value class instance
     */
    public boolean isFolded() {
        return retracter.get() == Value.kForward;
    }

    /**
     * set the new position of the retracter.
     * can be either Forward (Up) or Reverse (Down).
     *
     * @param direction the desired direction for the retracter
     */
    public void setPosition(Value direction) {
        retracter.set(direction);
    }

    /**
     * Toggle the position of the intake.
     * if you wish to change the position manually, use {@see setPosition(Value)} instead.
     */
    public void togglePosition() {
        if (isFolded()) {
            setPosition(Value.kReverse);
        } else
            setPosition(Value.kForward);
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
