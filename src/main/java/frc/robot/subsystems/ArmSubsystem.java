package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class ArmSubsystem extends SubsystemBase
{
    public enum ArmPosition_enum
    {
        stowed,
        shoot_podium,
        shoot_wing,
        climb_firstpos,
        climb_secondpos,
        amp,
        trap,
    }

    private static class ArmPose
    {
        public static final double ARM_EncoderCountsPer360 = 120000; //placeholder

        public static double PositionToRadians(double position)
        {
            double conversion = ARM_EncoderCountsPer360;
            return position / conversion * 2;
        }

        public static double PercentToPosition(double radians)
        {
            double conversion = ARM_EncoderCountsPer360;
            return radians * conversion / 2;
        }

        private static double _Arm;

        public ArmPose(double arm)
        {
            _Arm = arm;
        }

        public static double AwesomeArmPosition()
        {
            return PercentToPosition(_Arm);
        }

        //need a get function for armposition
    }

    private static final ArmSubsystem.ArmPose StowAngleFrontPose = new ArmSubsystem.ArmPose(0);

    private static final Map<ArmSubsystem.ArmPosition_enum, ArmSubsystem.ArmPose> Poses = Map.ofEntries(

            //Stowed
            Map.entry(ArmSubsystem.ArmPosition_enum.stowed,
                    new ArmSubsystem.ArmPose(0)
            ),

            //shoot_podium
            Map.entry(ArmSubsystem.ArmPosition_enum.shoot_podium,
                    new ArmSubsystem.ArmPose(0)
            ),

            //shoot_wing
            Map.entry(ArmSubsystem.ArmPosition_enum.shoot_wing,
                    new ArmSubsystem.ArmPose(0)
            ),

            //climb first position
            Map.entry(ArmSubsystem.ArmPosition_enum.climb_firstpos,
                    new ArmSubsystem.ArmPose(0)
            ),


            //climb second position
            Map.entry(ArmSubsystem.ArmPosition_enum.climb_secondpos,
                    new ArmSubsystem.ArmPose(0)
            ),

            //amp
            Map.entry(ArmSubsystem.ArmPosition_enum.amp,
                    new ArmSubsystem.ArmPose(0)
            ),

            //trap
            Map.entry(ArmSubsystem.ArmPosition_enum.trap,
                    new ArmSubsystem.ArmPose(0)
            )
    );

    private final boolean AlwaysStow = false;

    private TalonFX ArmMotor = new TalonFX(Constants.Arm.ARM_MOTOR_ID);

    private ArmPosition_enum DesiredPosition = ArmPosition_enum.stowed;

    public ArmSubsystem()
    {
        ConfigureMotors(ArmMotor,0, 0, 0, 0); //TODO: tune me
    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD)
    {

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = caclulateGravityFeedForward();
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

    private void runmotionMagic(AtomicReference<ArmPose> desiredPose)
    {
        final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

        m_motmag.withSlot(0);
        ArmMotor.setControl(m_motmag.withPosition(desiredPose.get().AwesomeArmPosition()));
    }


    private double caclulateGravityFeedForward()
    {
        double maxGravityFF = 0; //TODO: tune me
        double ticksper360 = ArmPose.ARM_EncoderCountsPer360;

        var rotorPosSignal = ArmMotor.getRotorPosition();
        rotorPosSignal.refresh();
        double currentPosition = rotorPosSignal.getValue();
//        var currentPositionLatency = rotorPosSignal.getTimestamp().getLatency();
        double AccurateCurrentPosition = currentPosition + ArmMotor.getClosedLoopError().getValue();
        double tickerPerDegree = ticksper360 / 360;
        double degrees = (AccurateCurrentPosition - (ticksper360 / 4)) / tickerPerDegree;
        double radians = Math.toRadians(degrees);
        double cosScalar = Math.cos(radians);

        return maxGravityFF * cosScalar;
    }


    public ArmPose GetPose(ArmPosition_enum position)
    {
        if (!Poses.containsKey(position))
        {
            System.out.println("Pose not found for position " + position.name());
            return new ArmPose(0);
        }

        return Poses.get(position);
    }

    public Command Command_SetPosition(ArmPosition_enum position)
    {
        return Commands.sequence(new PrintCommand(String.format("SetPosition %s\n", position.name()))
                , runOnce(() -> DesiredPosition = position)
                , Command_ActuateArmToDesired()
        );
    }

    private Command Command_ActuateArmToDesired()
    {
        AtomicReference<ArmPose> DesiredPose = new AtomicReference<>();


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
                                ArmAtTarget = Math.abs(ArmMotor.getClosedLoopReference().getValue() - DesiredPose.get().AwesomeArmPosition()) < 10;
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