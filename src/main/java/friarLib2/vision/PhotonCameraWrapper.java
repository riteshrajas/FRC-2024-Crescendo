package friarLib2.vision;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.common.hardware.VisionLEDMode;

public class PhotonCameraWrapper implements IVisionCamera {

    private PhotonCamera camera;

    /**
     * Initializes the PhotonCamera object 
     * 
     * @param name
     */
    public PhotonCameraWrapper (String name) {
        camera = new PhotonCamera(name);
    }

    /**
     * @return if the camera detects a target
     */
    @Override
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * @return an array of visionTarget objects
     */
    @Override
    public VisionTarget[] getTargets() {
        //Get the list of targets
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targetList = result.getTargets();

        //Parse the PhotonTrackedTarget into visionTarget
        VisionTarget[] targets = new VisionTarget[targetList.size()];
        for (int i = 0; i < targets.length; i++) {
            targets[i] = new VisionTarget(
                targetList.get(i).getYaw(), 
                targetList.get(i).getPitch(), 
                targetList.get(i).getArea(), 
                targetList.get(i).getSkew());
            }
        return targets;
    }

    /**
     * @return the object with index 0
     */
    @Override
    public VisionTarget getBestTarget() {
        try {
            return getTargets()[0];
        } catch (IndexOutOfBoundsException e) {
            return new VisionTarget();
        }
    }

    @Override
    public void setPipeline(int index) {
        if (index == 0) {
            camera.setDriverMode(true);
        } else {
            camera.setDriverMode(false);
            camera.setPipelineIndex(index);
        }
    }

    @Override
    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    @Override
    public void setLights(LedMode mode) {
        switch (mode) {
            case on: camera.setLED(VisionLEDMode.kOn); break;
            case off: camera.setLED(VisionLEDMode.kOff); break;
            case blink: camera.setLED(VisionLEDMode.kBlink); break;
            case currentPipeline: camera.setLED(VisionLEDMode.kDefault); break;
        }
    }

    @Override
    public int camMode() {
        return (int) NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").getNumber(0);
    }
}
