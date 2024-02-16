package friarLib2.vision;

import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Wrapper for the limelight's networktables API, which can be found at 
 * https://docs.limelightvision.io/en/latest/networktables_api.html
 */
public class LimelightCamera implements IVisionCamera {

    @Override
    public boolean hasTargets() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    @Override
    public VisionTarget[] getTargets() {

        VisionTarget target = new VisionTarget(
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0));

        VisionTarget[] targets = {target}; //Limelight only supports one target
        
        return targets;
    }

    @Override
    public VisionTarget getBestTarget() {
        return getTargets()[0];
    }

    @Override
    public int getPipeline() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getNumber(-1).intValue();
    }

    @Override
    public void setPipeline(int index) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(index);
    }

    @Override
    public void setLights(LedMode mode) {
        switch (mode) {
            case on: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); break;
            case off: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); break;
            case blink: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); break;
            case currentPipeline: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); break;
        }
    }

    @Override
    public int camMode() {
        return (int) NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").getNumber(0);
    }
}
