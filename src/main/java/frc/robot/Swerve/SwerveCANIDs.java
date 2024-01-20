package frc.robot.Swerve;

/**
 * Contains the CAN IDs for a single swerve module
 */
public class SwerveCANIDs {
    public int driveMotorID;
    public int steeringMotorID;
    public int CANCoderID;

    public SwerveCANIDs (int driveMotorID, int steeringMotorID, int CANCoderID) {
        this.driveMotorID = driveMotorID;
        this.steeringMotorID = steeringMotorID;
        this.CANCoderID = CANCoderID;
    }
}