package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.util.WPICleaner;
import edu.wpi.first.wpilibj2.command.*;
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
        Speaker,
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

        public double AwesomePivotPosition()
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
        ),

        Map.entry(PivotPosition_enum.Speaker,
            new PivotPose(0)
        )
    );

    private final boolean AlwaysStow = false;

    private PivotPosition_enum DesiredPosition = PivotPosition_enum.Stowed;

    private TalonFX PivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);
    private TalonFX IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    
    

    public IntakeSusbsystem()
    {
        ConfigureMotors(PivotMotor, 0, 0, 0, 0); //TODO: tune me
        ConfigureMotors(IntakeMotor, 0, 0, 0, 0); //TODO: tune me
    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD)
    {
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = new Slot0Configs();
//        slot0Configs.kS = Cacl
        slot0Configs.kV = kV;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

        // apply gains, 50 ms total timeout
        motor.getConfigurator().apply(slot0Configs, 0.050);
    }

    private void runmotionMagic(AtomicReference<PivotPose> desiredPose)
    {
        final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

        m_motmag.withSlot(0);
        PivotMotor.setControl(m_motmag.withPosition(desiredPose.get().AwesomePivotPosition()));
    }


    private double caclulateGravityFeedForward()
    {
        double maxGravityFF = 0; //TODO: tune me
        double ticksper360 = PivotPose.PIVOT_EncoderCountsPer360;

        var rotorPosSignal = PivotMotor.getRotorPosition();
        rotorPosSignal.refresh();
        double currentPosition = rotorPosSignal.getValue();
//        var currentPositionLatency = rotorPosSignal.getTimestamp().getLatency();
        double AccurateCurrentPosition = currentPosition + PivotMotor.getClosedLoopError().getValue();
        double tickerPerDegree = ticksper360 / 360;
        double degrees = (AccurateCurrentPosition - (ticksper360 / 4)) / tickerPerDegree;
        double radians = Math.toRadians(degrees);
        double cosScalar = Math.cos(radians);

        return maxGravityFF * cosScalar;
    }

    public boolean IntakehasNote()
    {
        return true; //TODO: finish code when sensors arrive
    }

    public Command Command_powerIntake(double IntakePower)
    {
        return run(() -> IntakeMotor.set(IntakePower));
    }

    public PivotPose GetPose(PivotPosition_enum position)
    {
        if (!Poses.containsKey(position))
        {
            System.out.println("Pose not found for position " + position.name());
            return new PivotPose(0);
        }

        return Poses.get(position);
    }

    public Command Command_SetPosition(PivotPosition_enum position)
    {
        return Commands.sequence(new PrintCommand(String.format("SetPosition %s\n", position.name()))
                , runOnce(() -> DesiredPosition = position)
                , Command_ActuatePivotToDesired()
        );
    }

    private Command Command_ActuatePivotToDesired()
    {
        AtomicReference<IntakeSusbsystem.PivotPose> DesiredPose = new AtomicReference<>();


        Command Move =
                run(() ->
                {
                    runmotionMagic(DesiredPose);
                })
                        .until(() ->
                        {
                            boolean ArmAtTarget = true;

                            {
                                //System.out.printf("AB %.0f -> %.0f\n", Motor_AB.getActiveTrajectoryPosition(), DesiredPose.get().UpperArmPosition());
                                ArmAtTarget = Math.abs(PivotMotor.getClosedLoopReference().getValue() - DesiredPose.get().AwesomePivotPosition()) < 10;
                            }
                            if (ArmAtTarget)
                            {
                                System.out.println("At Target");
                            }

                            return ArmAtTarget;
                        });

        return runOnce(() -> DesiredPose.set(GetPose(DesiredPosition)))
                .andThen(Move);

    }
}
