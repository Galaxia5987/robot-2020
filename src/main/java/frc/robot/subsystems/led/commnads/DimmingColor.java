package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

/**
 * Command that sets a dimming color.
 */
public class DimmingColor extends CommandBase {
    private final LED led;
    private final Color color;

    /**
     * Create a dimming color command.
     *
     * @param led the LED strip object
     */
    public DimmingColor(LED led, Color color) {
        this.led = led;
        this.color = color;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.activateDimness(true); // TODO: Test whether initialize is actually called in a default command
    }

    @Override
    public void execute() {
        led.setWholeStrip(color);
    }

    @Override
    public void end(boolean interrupted) {
        led.activateDimness(false);
    }
}
