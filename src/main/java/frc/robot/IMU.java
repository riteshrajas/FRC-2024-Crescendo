/*Can be reused if we use the same IMU */
package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Contains static references to the robot's internal measurement unit and related methods.
 */
public class IMU {

    // -- We're using a Pigeon 1.0 because we have a bunch floating around from previous robots
    //    The Pigeon 1 should be mounted Y-FORWARD
    public static PigeonIMU imu = new PigeonIMU(Constants.PIGEON_IMU_ID);

    /**
     * Set the yaw and FusedHeading to zero
     */
    public static void zeroIMU () {
        tareIMU(0);
    }

    /**
     * Set the yaw and FusedHeading to the specified value
     */
    public static void tareIMU (double newYawDegrees) {
        imu.setYaw(newYawDegrees);
        imu.setFusedHeading(newYawDegrees);
    }

    /**
     * Set the yaw and FusedHeading to rhe specified Rotation2d
     */
    public static void tareIMU (Rotation2d newYaw) {
        tareIMU(newYaw.getDegrees());
    }

    /**
     * Use the IMU to read the robot's yaw (left/right)
     * 
     * @return Rotation2d representing IMU's measured angle
     */
    public static Rotation2d getRobotYaw () {
        return Rotation2d.fromDegrees(imu.getFusedHeading());
    }

    /**
     * Use the IMU to read the robot's pitch (up/down)
     * 
     * @return Rotation2d representing IMU's measured angle
     */
    public static Rotation2d getRobotPitch () {
        return Rotation2d.fromDegrees(imu.getPitch());
    }

    /**
     * Use the IMU to read the robot's roll (side/side)
     * 
     * @return Rotation2d representing IMU's measured angle
     */
    public static Rotation2d getRobotRoll () {
        return Rotation2d.fromDegrees(imu.getRoll());
    }
}
    
