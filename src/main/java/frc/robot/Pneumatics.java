package frc.robot;

import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics {
    public static final Compressor COMPRESSOR = new Compressor(
        Constants.PCM_CAN_ID,
        Constants.PCM_TYPE
    );

    /**
     * @return The pressure in the robot's pneumatic tanks
     */
    public static double getStoragePSI() {
        return COMPRESSOR.getPressure();
    }

    /**
     * @return If the compressor is on (true) or off (false)
     */
    public static boolean getCompressorState() {
        return COMPRESSOR.isEnabled();
    }
}