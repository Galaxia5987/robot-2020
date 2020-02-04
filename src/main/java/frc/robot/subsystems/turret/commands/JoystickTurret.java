package frc.robot.subsystems.turret.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.turret.Turret;

import java.util.function.Supplier;

import static frc.robot.Constants.Turret.*;
public class JoystickTurret extends CommandBase {
    private static Turret turret;
    private Supplier<Double> joystickInput = OI::getXboxY;

    public JoystickTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double position = turret.getAngle() + joystickInput.get() * TURRET_JOYSTICK_SPEED;
        if (position < MINIMUM_POSITION && position > MAXIMUM_POSITION)
            turret.setAngle(position);
    }

}
