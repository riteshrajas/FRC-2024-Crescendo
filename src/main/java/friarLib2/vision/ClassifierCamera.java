package friarLib2.vision;

import edu.wpi.first.networktables.NetworkTableInstance;

//wrapper for the JSON Dump Specification of the Neural Classifier Results from the Networks

public class ClassifierCamera
{
    
    public static String ClassifierClass()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getString("null");
    }

   

}
