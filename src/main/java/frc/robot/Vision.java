package frc.robot;

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
}
