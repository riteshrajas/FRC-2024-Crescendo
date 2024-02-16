package friarLib2.hid;

import edu.wpi.first.wpilibj.Joystick;
import friarLib2.math.FriarMath;

/**
 * Represents a joystick, but with built in deadband calculation
 */
public class Joystick3309 extends Joystick
{
    public static final int LEFT_CLUSTER_1_ID = 11;
    public static final int LEFT_CLUSTER_2_ID = 12;
    public static final int LEFT_CLUSTER_3_ID = 13;
    public static final int LEFT_CLUSTER_4_ID = 14;
    public static final int LEFT_CLUSTER_5_ID = 15;
    public static final int LEFT_CLUSTER_6_ID = 16;
    public static final int RIGHT_CLUSTER_1_ID = 5;
    public static final int RIGHT_CLUSTER_2_ID = 6;
    public static final int RIGHT_CLUSTER_3_ID = 7;
    public static final int RIGHT_CLUSTER_4_ID = 8;
    public static final int RIGHT_CLUSTER_5_ID = 9;
    public static final int RIGHT_CLUSTER_6_ID = 10;
    
    
    
    private final double _Deadband;

    public Joystick3309 (final int port, double deadband)
    {
        super(port);
        this._Deadband = deadband;
    }
    
    public double GetXWithDeadband()
    {
        return ApplyDeadband(super.getX());
    }

    public double GetYWithDeadband()
    {
        return ApplyDeadband(super.getY());
    }
    
    public boolean GetCluster_Left_1() { return getRawButton(LEFT_CLUSTER_1_ID); }
    public boolean GetCluster_Left_2() { return getRawButton(LEFT_CLUSTER_2_ID); }
    public boolean GetCluster_Left_3() { return getRawButton(LEFT_CLUSTER_3_ID); }
    public boolean GetCluster_Left_4() { return getRawButton(LEFT_CLUSTER_4_ID); }
    public boolean GetCluster_Left_5() { return getRawButton(LEFT_CLUSTER_5_ID); }
    public boolean GetCluster_Left_6() { return getRawButton(LEFT_CLUSTER_6_ID); }
    public boolean GetCluster_Right_1() { return getRawButton(RIGHT_CLUSTER_1_ID); }
    public boolean GetCluster_Right_2() { return getRawButton(RIGHT_CLUSTER_2_ID); }
    public boolean GetCluster_Right_3() { return getRawButton(RIGHT_CLUSTER_3_ID); }
    public boolean GetCluster_Right_4() { return getRawButton(RIGHT_CLUSTER_4_ID); }
    public boolean GetCluster_Right_5() { return getRawButton(RIGHT_CLUSTER_5_ID); }
    public boolean GetCluster_Right_6() { return getRawButton(RIGHT_CLUSTER_6_ID); }
    
    public boolean GetCluster_Left_Top()
    {
        return GetCluster_Left_1() || GetCluster_Left_2() || GetCluster_Left_3();
    }
    
    public boolean GetCluster_Left_Bottom()
    {
        return GetCluster_Left_4() || GetCluster_Left_5() || GetCluster_Left_6();
    }
    
    public boolean GetCluster_Left_Any()
    {
        return GetCluster_Left_Top() || GetCluster_Left_Bottom();
    }

    public boolean GetCluster_Right_Top()
    {
        return GetCluster_Right_1() || GetCluster_Right_2() || GetCluster_Right_3();
    }

    public boolean GetCluster_Right_Bottom()
    {
        return GetCluster_Right_4() || GetCluster_Right_5() || GetCluster_Right_6();
    }

    public boolean GetCluster_Right_Any()
    {
        return GetCluster_Right_Top() || GetCluster_Right_Bottom();
    }
    
    
    
    private double ApplyDeadband(double value)
    {
        if (Math.abs(value) < _Deadband)
        {
            return 0;
        }
        
        return Math.signum(value) * FriarMath.Remap(Math.abs(value), _Deadband, 1, 0, 1);
    }
}
