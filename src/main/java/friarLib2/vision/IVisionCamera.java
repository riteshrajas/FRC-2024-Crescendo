package friarLib2.vision;

/**
 * A wrapper for a vision system. This is inteded to make it easy to switch between different vision
 * systems but keep the same API. For example, if we want to switch between PhotonVision and Limelight,
 * we can do so without needing to change too much code
 */
public interface IVisionCamera {
    boolean hasTargets ();
    VisionTarget[] getTargets ();
    VisionTarget getBestTarget ();
    int getPipeline ();
    void setPipeline (int index);
    void setLights (LedMode mode);
    int camMode ();
}
