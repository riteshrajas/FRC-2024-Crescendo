package friarLib2.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionCamera
{
    boolean hasTargets ();
    PhotonTrackedTarget[] getTargets ();
    PhotonTrackedTarget getBestTarget ();
    int getPipelineIndex ();
    void setPipelineIndex (int index);
    int camMode ();
    
}