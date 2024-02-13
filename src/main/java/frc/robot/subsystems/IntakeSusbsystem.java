package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.WPICleaner;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import friarLib2.utility.PIDParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.configs.jni.ConfigJNI;

import javax.naming.ldap.Control;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSusbsystem extends SubsystemBase {
    public enum EPivotPosition {
        stowed(0), // A
        shoot_podium(1),  // B
        intake(0),
        amp(0),
        trap(0);

        private final int Rotations;

        EPivotPosition(int rotations) {
            Rotations = rotations;
        }
    }

    private TalonFX PivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);
    private TalonFX IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    private final MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);

    private final VelocityDutyCycle Velocity = new VelocityDutyCycle(0);

    public IntakeSusbsystem() {
        ConfigureMotors(PivotMotor, 0, 0, 0, 0);
        ConfigureMotors(IntakeMotor, 0, 0, 0, 0);
        MotionMagic.Slot = 1;

    }

    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD) {

        var talonFXConfigs = new TalonFXConfiguration();

        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = caclulateGravityFeedForward();
        slot1Configs.kV = kV;
        slot1Configs.kP = kP;
        slot1Configs.kI = kI;
        slot1Configs.kD = kD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

        // apply gains, 50 ms total timeout
        motor.getConfigurator().apply(slot1Configs, 0.050);
    }

    private double caclulateGravityFeedForward() {
        double maxGravityFF = 0; //TODO: tune me
        double ticksper360 = 12000;

        double CurrentPosition = PivotMotor.getPosition().getValue();
//        var currentPositionLatency = rotorPosSignal.getTimestamp().getLatency();
        double AccurateCurrentPosition = CurrentPosition + PivotMotor.getClosedLoopError().getValue();
        double tickerPerDegree = ticksper360 / 360;
        double degrees = (AccurateCurrentPosition - (ticksper360 / 4)) / tickerPerDegree;
        double radians = Math.toRadians(degrees);
        double cosScalar = Math.cos(radians);

        return maxGravityFF * cosScalar;
    }

    public Command Command_SetIntakePosition(IntakeSusbsystem.EPivotPosition position) {
        return run(() ->
        {
            SmartDashboard.putNumber("Arm Target", position.Rotations);

            PivotMotor.setControl(MotionMagic.withPosition(position.Rotations));
        })
                .until(() ->
                {
                    double actualRotation = PivotMotor.getPosition().getValue();
                    SmartDashboard.putNumber("Arm Rotation", actualRotation);
                    return Math.abs(actualRotation - position.Rotations) < 0.10; // TODO: Tune this error threshold
                });
    }

    public Command Command_setIntakePower(double velocity)
    {
        return run(() ->
        {
            IntakeMotor.setControl(Velocity.withVelocity(velocity));
        })
            .until(() ->
            {
                return getCurrentCommand().isFinished();
            });

    }
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", PivotMotor.getPosition().getValue());
    }
}

