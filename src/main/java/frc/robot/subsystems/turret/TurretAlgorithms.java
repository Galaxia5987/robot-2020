package frc.robot.subsystems.turret;

public class TurretAlgorithms {

    /**
     * change the angle to the desired angle,
     * the value can be between -360 to 360 degrees.
     *
     * @param targetAngle the desired angle.
     * @return return the target angle.
     */
    public double setTurretAngle(double targetAngle, double currentPosition, double minPos, double maxPos) {
        targetAngle %= 360; targetAngle += 360; targetAngle %= 360; //Ensure that targetAngle is a number between 0-360.
        
        if (Math.abs(targetAngle+360-currentPosition) < Math.abs(targetAngle-360-currentPosition)) {
            if (Math.abs(targetAngle+360-currentPosition) < Math.abs(targetAngle-currentPosition)) {
                targetAngle += 360;
            }
        } else if (Math.abs(targetAngle-360-currentPosition) < Math.abs(targetAngle-currentPosition)) {
            targetAngle -= 360;
        }

        return targetAngle;
    }

    /**
     * @param minimum the minimum angle the turret can turn
     * @param angle the target angle
     * @param maximum the maximum angle that the turret can turn
     * @return an angle that satisfies the constrain
     */
    public double constrain(double minimum, double angle, double maximum) {
        return Math.min(maximum, Math.max(minimum, angle));
    }

    /**
     * @return the same position rotated 360 degrees, in order to have more room to rotate
     */
    public double center(double currentPosition, double minimum, double maximum) {
        double avg = (minimum+maximum)/2;
        if (currentPosition > (180 + avg)) {
            currentPosition -= 360;
        } else if (currentPosition < (-180 + avg)){
            currentPosition += 360;
        }
        return currentPosition;
    }

}

