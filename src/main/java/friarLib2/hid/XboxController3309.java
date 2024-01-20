package friarLib2.hid;

import edu.wpi.first.wpilibj.XboxController;
import friarLib2.math.FriarMath;

/**
 * Represents an Xbox controller, but with built in deadband calculation
 */
public class XboxController3309 extends XboxController {

    public double _Deadband;

    public XboxController3309 (final int port, double deadband)
    {
        super(port);
        this._Deadband = deadband;
    }

    @Override
    public double getLeftX() { return ApplyDeadband(super.getLeftX()); }

    @Override
    public double getLeftY() { return ApplyDeadband(super.getLeftY()); }

    @Override
    public double getRightX() { return ApplyDeadband(super.getRightX()); }

    @Override
    public double getRightY() { return ApplyDeadband(super.getRightY()); }

    public boolean LeftTrigger() { return getLeftTriggerAxis() > 0.5; }
    
    public boolean LeftTrigger(double threshold) { return getLeftTriggerAxis() > threshold; }

    public boolean RightTrigger() { return getRightTriggerAxis() > 0.5; }

    public boolean RightTrigger(double threshold) { return getRightTriggerAxis() > threshold; }
    
    public boolean DPad_Up() { return getPOV() == 0; }
    
    public boolean DPad_Right() { return getPOV() == 90; }
    
    public boolean DPad_Down() { return getPOV() == 180; }
    
    public boolean DPad_Left() { return getPOV() == 270; }
    
    
    private double ApplyDeadband(double value)
    {
        if (Math.abs(value) < _Deadband)
        {
            return 0;
        }

        return Math.signum(value) * FriarMath.Remap(Math.abs(value), _Deadband, 1, 0, 1);
    }
}
