package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Ports.Intake.MASTER;

public class Intake extends SubsystemBase {
    private VictorSPX master = new VictorSPX(MASTER);

    public Intake() {
        master.setInverted(MASTER_INVERTED);
    }
}
