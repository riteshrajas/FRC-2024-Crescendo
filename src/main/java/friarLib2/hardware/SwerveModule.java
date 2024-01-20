package friarLib2.hardware;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * An interface representing any swerve module
 */
public interface SwerveModule {
    public void setState (SwerveModuleState state);
    public SwerveModuleState getState ();
    public SwerveModulePosition getPosition ();
    public CANCoder GetCanCoder();

    default public void outputToDashboard () {}

    default public boolean steeringHasSlipped () { return false; }
    default public void zeroSteering () {}
    public void zeroPosition ();
}