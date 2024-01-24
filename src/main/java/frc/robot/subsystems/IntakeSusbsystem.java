package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.util.WPICleaner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import friarLib2.utility.PIDParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.configs.jni.ConfigJNI;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSusbsystem extends SubsystemBase 
{
    public enum PivotPosition_enum
    {
        Stowed,
        Intake,
        Amp,
        Trap,
        Index,
    }

    private static class PivotPose
    {
        public static final double PIVOT_EncoderCountsPer360 = 120000; //placeholder

        public static double PositionToRadians(double position)
        {
            double conversion = PIVOT_EncoderCountsPer360;
            return position / conversion * 2;
        }

        public static double PercentToPosition(double radians)
        {
            double conversion = PIVOT_EncoderCountsPer360;
            return radians * conversion / 2;
        }

        private double _Pivot;

        public PivotPose(double pivot)
        {
            _Pivot = pivot;
        }

        public double PivotPosition()
        {
            return PercentToPosition(_Pivot);
        }
    }

    private static final PivotPose StowAngleFrontPose = new PivotPose(0); 

    private static final Map<PivotPosition_enum, PivotPose> Poses = Map.ofEntries(

        //Stowed
        Map.entry(PivotPosition_enum.Stowed,
            new PivotPose(0)
        ),

        //Intake
        Map.entry(PivotPosition_enum.Intake,
            new PivotPose(0)
        ),

        //Amp
        Map.entry(PivotPosition_enum.Amp,
            new PivotPose(0)
        ),

        //Trap
        Map.entry(PivotPosition_enum.Trap,
            new PivotPose(0)
        ),


        //Index
        Map.entry(PivotPosition_enum.Index,
            new PivotPose(0)
        )
    );

    private final boolean AlwaysStow = false;

    private TalonFX PivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);
    private TalonFX IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    
    

    public IntakeSusbsystem()
    {
        ConfigureMotors(PivotMotor, 0, 0, 0, 0); //TODO: tune me
        ConfigureMotors(IntakeMotor, 0, 0, 0, 0); //TODO: tune me
    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD)
    {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = kV;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        // apply gains, 50 ms total timeout
        motor.getConfigurator().apply(slot0Configs, 0.050);
    }

    

    public boolean IntakehasNote()
    {
        return true; //TODO: finish code when sensors arrive
    }



    public Command Command_powerIntake(double IntakePower)
    {
        return run(() -> IntakeMotor.set(IntakePower));
    }

    
    







    



    
}
