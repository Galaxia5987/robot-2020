package frc.robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climb.Climber;

public class ToggleStopper extends InstantCommand {
    private final Climber climber;

    public ToggleStopper(Climber climber) {
        addRequirements(climber);
        this.climber = climber;
    }

    @Override
    public void initialize() {
        if(climber.isStopperEngaged()) climber.releaseStopper();
        else climber.engageStopper();
    }
}
