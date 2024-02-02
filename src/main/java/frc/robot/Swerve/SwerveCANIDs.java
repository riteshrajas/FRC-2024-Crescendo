package frc.robot.Swerve;

/**
 * Contains the CAN IDs for a single swerve module
 */
public class SwerveCANIDs {
    public int driveMotorID;
    public int steeringMotorID;
    public int CANcoderID;

    public SwerveCANIDs (int driveMotorID, int steeringMotorID, int CANcoderID) {
        this.driveMotorID = driveMotorID;
        this.steeringMotorID = steeringMotorID;
        this.CANcoderID = CANcoderID;
    }
}