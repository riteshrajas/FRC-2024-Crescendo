package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    public enum EArmPosition {
        stowed(0),
        shoot_subwoofer(1),
        shoot_podium(0), //TODO: tune when added
        shoot_wing(2), //TODO: tune when added
        climb_firstpos(10),
        climb_secondpos(0),
        amp(0),
        trap(0);

        private final int EncoderCount;

        EArmPosition(int encoderCount) {
            EncoderCount = encoderCount;
        }
    }

    private TalonFX LeftArmMotor = new TalonFX(Constants.Arm.ARM_MOTOR_LEFT);
    private TalonFX RightArmMotor = new TalonFX(Constants.Arm.ARM_MOTOR_RIGHT);
    private final MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);

    public ArmSubsystem() {
        ConfigureMotors(LeftArmMotor, 0, 0, 0, 0);
        ConfigureMotors(RightArmMotor, 0, 0, 0, 0);
        RightArmMotor.setControl(new Follower(Constants.Arm.ARM_MOTOR_LEFT, false));
        MotionMagic.Slot = 0;

    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD) {

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; //GFF
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
        motor.setPosition(0);
    }

    private double caclulateGravityFeedForward()
    {
        double maxGravityFF = 0; //TODO: tune me
        double ticksper360 = 12000;

        double CurrentPosition = LeftArmMotor.getPosition().getValue();
//        var currentPositionLatency = rotorPosSignal.getTimestamp().getLatency();
        double AccurateCurrentPosition = CurrentPosition + LeftArmMotor.getClosedLoopError().getValue();
        double tickerPerDegree = ticksper360 / 360;
        double degrees = (AccurateCurrentPosition - (ticksper360 / 4)) / tickerPerDegree;
        double radians = Math.toRadians(degrees);
        double cosScalar = Math.cos(radians);

        return maxGravityFF * cosScalar;
    }

    public Command Command_SetPosition(EArmPosition position) {
        return run(() ->
        {
            SmartDashboard.putNumber("Arm Target", position.EncoderCount);

            LeftArmMotor.setControl(MotionMagic.withPosition(position.EncoderCount));
        })
                .until(() ->
                {
                    double actualRotation = LeftArmMotor.getPosition().getValue();
                    SmartDashboard.putNumber("Arm Rotation", actualRotation);
                    return Math.abs(actualRotation - position.EncoderCount) < 0.10; // TODO: Tune this error threshold
                });
    }

    public Command zeroArmEncoder()
    {
       return runOnce(() -> LeftArmMotor.setPosition(0));
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", LeftArmMotor.getPosition().getValue());
    }
}