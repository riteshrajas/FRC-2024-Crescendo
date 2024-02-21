package frc.robot.subsystems.VisionSystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.VisionSystems.*;
import friarLib2.vision.VisionIO;

public class VisionSubsystem extends SubsystemBase
{
    private static final double lowest_distance = Units.feetToMeters(10.0);

    private final VisionIO[] cameras;

    private int acceptableTagID;
    private boolean useSingleTag = false;

    private final List<VisionSubsystem.PoseAndTimestamp> results = new ArrayList<>();

    public VisionSubsystem(VisionIO[] cameras)
    {
        this.cameras = cameras;
    }

    final class PoseAndTimestamp
    {
        Pose2d pose;
        double timestamp;
    }

    
    
}