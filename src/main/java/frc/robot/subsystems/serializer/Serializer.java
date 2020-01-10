package frc.robot.subsystems.serializer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Serializer.*;

public class Serializer extends SubsystemBase {
    private TalonSRX serializerExitMotor = new TalonSRX(EXIT_MOTOR);
    private VictorSPX serializerEntryMotor = new VictorSPX(ENTRY_MOTOR);
    private AnalogInput entryProximity = new AnalogInput(ENTRY_PROXIMITY);
    private AnalogInput middleProximity = new AnalogInput(MIDDLE_PROXIMITY);
    private AnalogInput exitProximity = new AnalogInput(EXIT_PROXIMITY);

    public Serializer() {

    }
}
