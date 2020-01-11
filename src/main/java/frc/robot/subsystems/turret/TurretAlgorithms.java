package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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
        double[] positions = {targetAngle-360, targetAngle, targetAngle+360};
        double targetPosition = Double.NaN;
        double shortestDistance = Double.MAX_VALUE;
        for (double _targetPos: positions){
            if(_targetPos < minPos || targetPosition > maxPos)
                continue;
            if(Math.abs(_targetPos - currentPosition) < shortestDistance)
            {
                shortestDistance = Math.abs(_targetPos - currentPosition);
                targetPosition = _targetPos;
            }
        }
        return targetPosition;
    }

    /**
     * @param minimum the minimum angle the turret can turn
     * @param angle the target angle
     * @param maximum the maximum angle that the turret can turn
     * @return an angle that satisfies the constrain
     */
    public double boundary(double minimum, double angle, double maximum) {
        if (angle < minimum) {
            angle -= 360;
        } else if (angle > maximum) {
            angle += 360;
        }
        return angle;
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

