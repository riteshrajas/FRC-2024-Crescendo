package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import friarLib2.vision.LimelightCamera;
import friarLib2.vision.PhotonCameraWrapper;
import friarLib2.vision.IVisionCamera;
import friarLib2.vision.utility.PixelToAngle;


/**
 * Container for the limelightvision systems
 */
public class LimelightVision {

    public static Command TogglePipelineCommand()
    {
        return Commands.runOnce( () -> shooterCamera.setPipeline(1 - shooterCamera.getPipeline()) );
    }

    public static Command SetPipelineCommand(int index)
    {
        return Commands.runOnce( () -> shooterCamera.setPipeline(index) );
    }
    
    
    public static IVisionCamera shooterCamera = new LimelightCamera();

    private static final PixelToAngle ANGLE_CONVERTER = new PixelToAngle(320, 240, 54, 41); // Constants for the limelight 2
    private static final double HEIGHT_OF_CAMERA = 49.61; // inches
    private static final double ANGLE_OF_CAMERA = 160; // Degrees

    private static double lastDistance = 0; // Return this if the robot does not have a target

    /**
     * @return the distance in meters from the target
     */
    public static double getInchesFromTarget () {
        try {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry ty = table.getEntry("ty");
            double a2 = ty.getDouble(0.0);
            double a1 = ANGLE_OF_CAMERA;
            double h1 = HEIGHT_OF_CAMERA;
            double h2 = 26; // Place holder Height of target

            double angleToGoal = (a1 + a2);
            double angleToGoalRadian = Math.toRadians(angleToGoal);
            lastDistance = (h2-h1) / Math.tan(angleToGoalRadian);

        } catch (IndexOutOfBoundsException e) {} // If the camera has no target the target has no 
        return lastDistance;
    }
}
