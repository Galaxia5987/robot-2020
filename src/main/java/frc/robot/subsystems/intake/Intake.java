package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Ports.Intake.MASTER;

public class Intake implements Subsystem {
    private VictorSPX master = new VictorSPX(MASTER);
}
