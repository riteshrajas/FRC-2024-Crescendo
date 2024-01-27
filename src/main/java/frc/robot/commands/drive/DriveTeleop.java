/*This NEEDS to be rewriten */
package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.IMU;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;
import friarLib2.math.FriarMath;
import friarLib2.utility.DoubleSlewRateLimiter;
import friarLib2.utility.Vector3309;

/**
 * Use the left joystick position to control the robot's direction of 
 * travel relative to the field. Robot heading change is determined 
 * by moving the right joystick left and right.
 * 
 * <p>
 * i.e., the robot will move in whatever direction the stick is pushed,
 * regardless of its orientation on te field.
 * 
 * <p>
 * This class is designed to be subclassed so that other commands can 
 * use the same translational velcoity calculations for field-relative 
 * teleop control while being able to set the rotational speed 
 * independently (based on vision data for example).
 */
public class DriveTeleop extends Command {

    private static final double OutputRange = 0.2;


    protected DriveSubsystem drive;

    //private static final DoubleSlewRateLimiter xAccelLimiter = new DoubleSlewRateLimiter(Constants.Drive.MAX_TELEOP_ACCELERATION, Constants.Drive.MAX_TELEOP_DECELERATION);
    //private static final DoubleSlewRateLimiter yAccelLimiter = new DoubleSlewRateLimiter(Constants.Drive.MAX_TELEOP_ACCELERATION, Constants.Drive.MAX_TELEOP_DECELERATION);

    private static final SlewRateLimiter xAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_TELEOP_ACCELERATION);
    private static final SlewRateLimiter yAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_TELEOP_ACCELERATION);

    /** Menu on the dashboard to toggle acceleration limits */
    static SendableChooser<Boolean> accelChooser = new SendableChooser<>();
    static {
        // Configure the dashboard menu
        accelChooser.setDefaultOption("Normal acceleration", true);
        accelChooser.addOption("No acceleration limits", false);
        SmartDashboard.putData("Drivetrain acceleration", accelChooser);
    }

    public DriveTeleop(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        
        double x = OI.DriverLeft.GetXWithDeadband();
        double y = OI.DriverLeft.GetYWithDeadband();

        if (OI.DriverRight.getTrigger())
        {
            x = FriarMath.Remap(x, -1, 1, -OutputRange, OutputRange);
            y = FriarMath.Remap(y, -1, 1, -OutputRange, OutputRange);
        }

       
        Vector3309 translationalSpeeds = Vector3309.fromCartesianCoords(-x, -y)
                .capMagnitude(1)
                .scale(Constants.Drive.MAX_TELEOP_SPEED);

        if (accelChooser.getSelected()) {
            // Limit the drivebase's acceleration to reduce wear on the swerve modules
            translationalSpeeds.setXComponent(xAccelLimiter.calculate(translationalSpeeds.getXComponent()));
            translationalSpeeds.setYComponent(yAccelLimiter.calculate(translationalSpeeds.getYComponent()));
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translationalSpeeds.getXComponent(), 
            translationalSpeeds.getYComponent(), 
            calculateRotationalSpeed(translationalSpeeds), 
            IMU.getRobotYaw());

        drive.setChassisSpeeds(speeds);


    }


    /**
     * Given the field-relative translational speeds requested by the
     * operators, calculate the rotational speed of the robot.
     * 
     * @param translationalSpeeds
     * @return The rotational speed in radians/second
     */
    protected double calculateRotationalSpeed (Vector3309 translationalSpeeds)
    {
        double x = OI.DriverRight.GetXWithDeadband();

        if (OI.DriverRight.getTrigger())
        {
            x = FriarMath.Remap(x, -1, 1, -OutputRange, OutputRange);
        }
        
        return Constants.Drive.MAX_TELEOP_ROTATIONAL_SPEED * -x;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
    
