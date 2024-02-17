package frc.robot.subsystems.VisionSystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import friarLib2.vision.VisionTarget;

public class PhotonVision
{
    
    public void PhotonTarget()
    {
        int x = 0;
        int y = 0;
        int area = 0;
        int skew = 0;
        Transform2d pose = new Transform2d();
    }

    public static Command togglePipelineCommand()
    {
        return Commands.runOnce(() -> testCamera.setPipelineIndex(1 - testCamera.getPipelineIndex()));
    }

    public static Command setPipelineCommand(int index)
    {
        return Commands.runOnce(() -> testCamera.setPipelineIndex(index));
    }

    public static PhotonCamera testCamera = new PhotonCamera("testCamera");

    private static final double heightOfCamera = 49.61; // Height in inches of camera
    private static final double angleOfCamera = 10; // Angle in degrees

    private static double lastDistance = 0; // Returns zero if no targets found

    public static double returnInchesFromTarget()
    {
        try
        {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("photonCamera");
            NetworkTableEntry tv = table.getEntry("tv");

            double a2 = tv.getDouble(0.0);
            double a1 = angleOfCamera;
            double h1 = heightOfCamera;
            double h2 = 57.13;

            double angleToGoal = (a1 + a2);
            double angleToGoalRadian = Math.toRadians(angleToGoal);

            lastDistance = (h2-h1) / Math.tan(angleToGoalRadian);
        } catch (IndexOutOfBoundsException e){}

        return lastDistance;
    }

    public static PhotonTrackedTarget getBestTarget()
    {
        var result = testCamera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        return target;   
    }
}
