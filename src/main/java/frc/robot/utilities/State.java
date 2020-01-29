package frc.robot.utilities;

/**
 * This is a state enum for the pneumatics of the robot.
 * Each mechanism has its own OPEN and CLOSE states, and the option of a TOGGLE, which does not have to be handled.
 * The usage of this enum is pretty logical in its names but if there is doubt it goes as follows:
 * In the intake - CLOSE means the mechanism is folded.              OPEN means the mechanism is open.
 * In the conveyor - CLOSE means that the balls are being stopped.   OPEN means the stopper is open.
 * In the climb - CLOSE means that the climb is being stopped.       OPEN means the breaks are open.
 * the drivetrain won't implement this class, it won't make it easier to read.
 */
public enum State {
    OPEN,
    CLOSE,
    TOGGLE;
}
