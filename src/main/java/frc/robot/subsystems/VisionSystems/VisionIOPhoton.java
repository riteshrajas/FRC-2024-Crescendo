package frc.robot.subsystems.VisionSystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import friarLib2.util.constants.FieldConstants;
import friarLib2.vision.VisionIO;

public class VisionIOPhoton implements VisionIO
{
    
    private final PhotonCamera camera;
    private final String name;
    private final PhotonPoseEstimator odometry;
    private double pastTimestamp;
    private Transform3d pose;
    public List<PhotonTrackedTarget> targets;
    private static DriverStation.Alliance storedAlliance = DriverStation.Alliance.Blue;

    /**
     * Implements PhotonVision camera
     * 
     * @param name Name of the camera
     * @param pose Location of the camera
     * 
     */
    public VisionIOPhoton(String name, Transform3d pose)
    {
        this.name = name;
        this.pose = pose;
        camera = new PhotonCamera(name);

        odometry = new PhotonPoseEstimator(FieldConstants.blue_field_layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO, camera, pose);
        odometry.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        PhotonPipelineResult result = camera.getLatestResult();
        Optional<EstimatedRobotPose> currentPose = checkValidResult(result.targets) ? odometry.update(result) : Optional.empty();

        // set if PhotonVision has a target
        if (result.hasTargets() && currentPose.isPresent())
        {
            inputs.hasTarget = true;
            targets = currentPose.get().targetsUsed;
        } else {
            inputs.hasTarget = false;
        }

        inputs.isNew = false;

        if (currentPose.isPresent() && targets != null)
        {
            if (targets.size() > 1)
            {
                double minDistance = Double.MAX_VALUE;
                double maxDistance = 0.0;
                double maxAmbiguity = 0.0;
                for (PhotonTrackedTarget target : targets)
                {
                    double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                    double ambiguity = target.getPoseAmbiguity();
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                    if (distance > minDistance)
                    {
                        maxDistance = distance;
                    }
                    if (ambiguity > maxAmbiguity)
                    {
                        maxAmbiguity = ambiguity;
                    }
                }
                inputs.minDistance = minDistance;
                inputs.maxDistance = maxDistance;
                inputs.maxAmbiguity = maxAmbiguity;

            } else {
                inputs.minDistance = targets.get(0).getBestCameraToTarget().getTranslation().getNorm();
                inputs.maxDistance = inputs.minDistance;
                inputs.maxAmbiguity = targets.get(0).getPoseAmbiguity();
            }

            inputs.singleIdUsed = targets.get(0).getFiducialId();
            inputs.singleIdUsedDouble = inputs.singleIdUsed;

            if (inputs.maxAmbiguity < 0.7)
            {
                inputs.timestamp = currentPose.get().timestampSeconds;

                if (pastTimestamp != inputs.timestamp)
                {
                    inputs.isNew = true;
                }

                inputs.x = currentPose.get().estimatedPose.getX();
                inputs.y = currentPose.get().estimatedPose.getY();
                inputs.rotation = currentPose.get().estimatedPose.getRotation().getAngle();
            }
        }

        pastTimestamp = inputs.timestamp;

        odometry.setFieldTags(FieldConstants.getFieldLayout());
    }

    @Override
    public String getName()
    {
        return name;
    }

    @Override
    public void setReferencePose(Pose2d pose)
    {
        odometry.setReferencePose(pose);
    }

    private boolean checkValidResult(List<PhotonTrackedTarget> result)
    {
        for (PhotonTrackedTarget target : result)
        {
            if (target.getFiducialId() > 8)
            {
                return false;
            }
        }
        return true;
    }

}
