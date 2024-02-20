package friarLib2.vision;

import edu.wpi.first.math.geometry.Pose2d;


public interface VisionIO
{   
    class VisionIOInputs
    {
        public double x;
        public double y;
        public double rotation;
        public double timestamp;
        public boolean isNew; // New pose

        public double maxAmbiguity;
        public double maxDistance;
        public double minDistance;
        
        public boolean hasTarget = false;
        public int singleIdUsed;
        public double singleIdUsedDouble;

        public double translationToTargetX;
        public double translationToTargetY;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default String getName()
    {
        return "";
    }

    default void setReferencePose(Pose2d pose) {}
}
