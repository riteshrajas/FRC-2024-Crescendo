package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

public class Vision
{
    static private final Set<Integer> ValidTags = Set.of
        (
              1, 2       // Blue Source
            , 4          // Red Speaker (center)
            , 5          // Red Amp
            , 6          // Blue Amp
            , 7          // Blue Speaker (center)
            , 9, 10      // Red Source
            , 11, 12, 13 // Red Stage
            , 14, 15, 16 // Red Stage
        );

    static private boolean ResultsAreStale = true;
    static private LimelightHelpers.Results LatestResults = null;


    static public void Periodic()
    {
        ResultsAreStale = true;
    }


    static private void UpdateResults()
    {
        if (ResultsAreStale)
        {
            LatestResults = LimelightHelpers.getLatestResults("").targetingResults;
            ResultsAreStale = false;
        }
    }

    static public LimelightHelpers.LimelightTarget_Fiducial GetBestTarget()
    {
        UpdateResults();

        LimelightHelpers.LimelightTarget_Fiducial bestTarget = null;

        // -- loop through all fiducial targets and find the valid target that's closest to the center
        for (LimelightHelpers.LimelightTarget_Fiducial target : LatestResults.targets_Fiducials)
        {
            if (ValidTags.contains((int)target.fiducialID))
            {
                if (bestTarget == null || Math.abs(target.tx) < Math.abs(bestTarget.tx))
                {
                    bestTarget = target;
                }
            }
        }


        return bestTarget;
    }

    static public LimelightHelpers.LimelightTarget_Detector GetBestNoteTarget()
    {
        LimelightHelpers.LimelightTarget_Detector bestNoteTarget = null;

        for (LimelightHelpers.LimelightTarget_Detector target : LatestResults.targets_Detector)
        {
            if (bestNoteTarget == null || Math.abs(target.tx) < Math.abs(bestNoteTarget.tx))
            {
                bestNoteTarget = target;
            }
        }
        return bestNoteTarget;
    }

    static public double GetDistanceFromTarget()
    {
        var target = GetBestTarget();
        if (target == null){ return 0; }

        double OffsetAngle = 0.0;

        double LimelightMountHeight = 24.5;

        double LimelightMountAngle = 13.0;

        double GoalHeight = 0;

        if (ValidTags.contains((int)target.fiducialID))
        {
            if (target.fiducialID == 5 || target.fiducialID == 6)
            {
                GoalHeight = 53.38;
            }
            else if (target.fiducialID == 4 || target.fiducialID == 7)
            {
                GoalHeight = 57.13;
            }
        }

        double AngleToGoalRadians = LimelightMountAngle + OffsetAngle;
        double AngleToGoalDegrees = AngleToGoalRadians * (Math.PI / 180);

        double distance = (GoalHeight - LimelightMountHeight) / Math.tan(LimelightMountAngle + AngleToGoalDegrees);

        return distance;
    }

    public static void OutputData()
    {
        var distance = GetDistanceFromTarget();
        var target = GetBestTarget();

        SmartDashboard.putNumber("Distance from Target", distance);
    }

}
