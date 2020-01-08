package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.MASTER_INVERTED;
import static frc.robot.Ports.Intake.*;
import static frc.robot.Robot.intake;

public class Intake extends SubsystemBase {
    private VictorSPX masterMotor = new VictorSPX(MASTER);
    private DoubleSolenoid solenoid = new DoubleSolenoid(SOLENOID_FORWARD_CHANNEL, SOLENOID_BACKWARD_CHANNEL);

    public Intake() {
        masterMotor.setInverted(MASTER_INVERTED);
    }

    public void togglePosition() {
        if (intake.getPosition() == Value.kForward) {
            setPosition(Value.kReverse);
        } else if (intake.getPosition() == Value.kReverse)
            setPosition(Value.kForward);
    }

    public Value getPosition() {
        return solenoid.get();
    }

    /**
     * set
     *
     * @param direction
     */
    public void setPosition(Value direction) {
        solenoid.set(direction);
    }

    public void applyPowerOnWheels(double speed) {
        masterMotor.set(ControlMode.PercentOutput, speed);
    }
}
