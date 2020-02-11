package frc.robot.subsystems.turret.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.turret.Turret;

import java.util.function.Supplier;
import static frc.robot.Constants.Turret.MAXIMUM_POSITION;
import static frc.robot.Constants.Turret.MINIMUM_POSITION;
import static frc.robot.Constants.Turret.*;

public class JoystickTurret extends CommandBase {
    private static Turret turret;
    private Supplier<Double> joystickInput = OI::getXboxLY;
    public static final double JOYSTICK_DEADBAND = 0.08;

    public JoystickTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double input = joystickInput.get();
        if(Math.abs(input) < JOYSTICK_DEADBAND) return;
        double position = turret.getAngle() + input * TURRET_JOYSTICK_SPEED;
        if (position >= MINIMUM_POSITION && position <= MAXIMUM_POSITION)
            turret.setAnglePosition(position);
    }

}
