package friarLib2.hardware;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * An interface representing any swerve module
 */
public interface SwerveModule {
    void setState(SwerveModuleState state);
    SwerveModuleState getState();
    SwerveModulePosition getPosition();
    CANcoder GetCanCoder();

    default void outputToDashboard() {}

    default boolean steeringHasSlipped() { return false; }
    default void zeroSteering(int encoderOffset) {}
    void zeroPosition();
}