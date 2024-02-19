package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSusbsystem extends SubsystemBase {
    public enum EPivotPosition {
        stowed(0), // A
        shoot_speaker(1),  // B
        intake(0),
        amp(0),
        trap(0);

        private final int Rotations;

        EPivotPosition(int rotations) {
            Rotations = rotations;
        }
    }

    public enum EOutakeType
    {
        amp(0),
        speaker(0),
        trap(0);

        private final int RPM;

        EOutakeType(int rpm){ RPM = rpm; }
    }

    private TalonFX PivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);

    private SparkPIDController IntakePID;
    private CANSparkFlex IntakeMotor;
    private RelativeEncoder IntakeEncoder;
//    private TalonFX IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);

    private final MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);

//    private final VelocityDutyCycle Velocity = new VelocityDutyCycle(0);

    public IntakeSusbsystem() {
        ConfigureMotors(PivotMotor, 0, 0, 0, 0);
        IntakeMotor = CreateSparkMotor(Constants.Intake.INTAKE_MOTOR_ID);
        IntakePID = CreatePID(IntakeMotor);
//        ConfigureMotors(IntakeMotor, 0, 0, 0, 0);
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
        motor.setPosition(0);
    }
    private CANSparkFlex CreateSparkMotor(int motorId) {
        CANSparkFlex motor = new CANSparkFlex(motorId, CANSparkLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        return motor;
    }
    private SparkPIDController CreatePID(CANSparkFlex motor) {
        var pid = motor.getPIDController();

        pid.setP(0.1);
        pid.setI(0);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0);
        pid.setOutputRange(-1, 1);

        pid.setSmartMotionMaxVelocity(1000, 0);
        pid.setSmartMotionMaxAccel(500, 0);
        pid.setSmartMotionMinOutputVelocity(0, 0);
        pid.setSmartMotionAllowedClosedLoopError(0.0021, 0);


        return pid;
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

    public Command Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition position) {
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

    public Command Command_IntakeNote()
    {
        return
            startEnd(
                () -> { IntakePID.setReference(1, CANSparkBase.ControlType.kSmartVelocity, 0); },
                () -> { IntakePID.setReference(0, CANSparkBase.ControlType.kSmartVelocity, 0); }
            )
            .until(() ->
            {
                //TODO: check for limit switch
                return false;
            });
    }

    public Command Command_OuttakeNote(EOutakeType outakeType)
    {
        return
            startEnd(
                () -> { IntakePID.setReference(outakeType.RPM, CANSparkBase.ControlType.kSmartVelocity, 0); },
                () -> { IntakePID.setReference(0, CANSparkBase.ControlType.kSmartVelocity, 0); }
            );
    }



    public Command zeroPivotEncoder()
    {
        return runOnce(() -> PivotMotor.setPosition(0));
    }

    public void periodic() {
        SmartDashboard.putNumber("pivot Position", PivotMotor.getPosition().getValue());
    }
}

