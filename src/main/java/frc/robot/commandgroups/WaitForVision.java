package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.VisionModule;

public class WaitForVision extends CommandBase {

    @Override
    public boolean isFinished() {
        return VisionModule.targetSeen();
    }
}
