package friarLib2.util.constants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants
{
    public static final double field_length_m = Units.feetToMeters(54.435);
    public static AprilTagFieldLayout blue_field_layout;
    public static AprilTagFieldLayout red_field_layout;
    private static DriverStation.Alliance storedAlliance = DriverStation.Alliance.Red;

    static
    {
        try
        {
            blue_field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            blue_field_layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            red_field_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            red_field_layout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }

    }

    public static final double amp_opening_center_from_apriltag = Units.inchesToMeters(-6);

    public static final double speaker_opening_center_from_apriltag = Units.inchesToMeters(18);

    public static final double feeder_opening_from_apriltag = Units.inchesToMeters(0); // TODO: set distance

}
