package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

import javax.swing.*;
import java.util.Objects;

import static edu.wpi.first.math.util.Units.radiansToRotations;

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

        private final int Rotations;

        EArmPosition(int rotations) {
            Rotations = rotations;
        }
    }

    private TalonFX LeftArmMotor = new TalonFX(Constants.Arm.ARM_MOTOR_LEFT);
    private TalonFX RightArmMotor = new TalonFX(Constants.Arm.ARM_MOTOR_RIGHT);
    private final MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);

    public ArmSubsystem() {
        ConfigureMotors(LeftArmMotor, 0, 0, 0, 0);
        ConfigureMotors(RightArmMotor, 0, 0, 0, 0);
        RightArmMotor.setControl(new Follower(23, false));
        MotionMagic.Slot = 0;

    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD) {

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

    private double caclulateGravityFeedForward() {
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
            SmartDashboard.putNumber("Arm Target", position.Rotations);

            LeftArmMotor.setControl(MotionMagic.withPosition(position.Rotations));
        })
                .until(() ->
                {
                    double actualRotation = LeftArmMotor.getPosition().getValue();
                    SmartDashboard.putNumber("Arm Rotation", actualRotation);
                    return Math.abs(actualRotation - position.Rotations) < 0.10; // TODO: Tune this error threshold
                });
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", LeftArmMotor.getPosition().getValue());
    }
}