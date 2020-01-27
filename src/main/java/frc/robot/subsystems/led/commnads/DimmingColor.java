package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

/**
 * Command that sets a dimming color.
 */
public class DimmingColor extends CommandBase {
    private final LED ledStrip;
    private final Color color;

    /**
     * Create a dimming color command.
     *
     * @param ledStrip the LED strip object
     */
    public DimmingColor(LED ledStrip, Color color) {
        this.ledStrip = ledStrip;
        this.color = color;
        addRequirements(ledStrip);
    }

    @Override
    public void initialize() {
        ledStrip.activateDimness(true); // TODO: Test whether initialize is actually called in a default command
    }

    @Override
    public void execute() {
        ledStrip.setWholeStrip(color);
    }

    @Override
    public void end(boolean interrupted) {
        ledStrip.activateDimness(false);
    }
}
