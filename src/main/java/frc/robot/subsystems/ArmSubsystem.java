package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
        stowed(0), // A
        shoot_podium(1),  // B
        shoot_wing(2),  // X
        climb_firstpos(10),  // Y
        climb_secondpos(0),
        amp(0),
        trap(0);

        private final int Rotations;

        EArmPosition(int rotations) {
            Rotations = rotations;
        }
    }

    private CANSparkFlex ArmMotorLeft;
    private CANSparkFlex ArmMotorRight;

    private SparkPIDController ArmPID;
    private RelativeEncoder ArmInternalEncoder;
    private RelativeEncoder ArmExternalEncoder; // Through Bore Encoder
    private double setpoint;

    private TrapezoidProfile profile;
    private Timer timer;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;
    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;



    private final double FF = 0.2;


    public ArmSubsystem() {
        ArmMotorLeft = CreateMotor(Constants.Arm.ARM_MOTOR_LEFT);
        ArmMotorRight = CreateMotor(Constants.Arm.ARM_MOTOR_RIGHT);
        ArmMotorRight.follow(ArmMotorLeft);

        ArmPID = CreatePID(ArmMotorLeft);
//        ArmInternalEncoder = ArmMotorLeft.getEncoder();
        ArmExternalEncoder = ArmMotorLeft.getExternalEncoder(8192);
        ArmInternalEncoder = ArmMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
//        ArmInternalEncoder.setPositionConversionFactor();
//        ArmInternalEncoder.setVelocityConversionFactor();
        ArmInternalEncoder.setPosition(0.0);

        ArmMotorLeft.burnFlash();
        ArmMotorRight.burnFlash();

        setpoint = EArmPosition.stowed.Rotations;

        timer = new Timer();
        timer.start();

        updateMotionprofile();




        // Uncomment this out when we have the Through Bore Encoder connected (and redo positions)
        // Also switch any other uses of ArmInternalEncoder to ArmExternalEncoder
        // ArmPID.setFeedbackDevice(ArmExternalEncoder);
    }
    private void updateMotionprofile()
    {
        startState = new TrapezoidProfile.State(ArmInternalEncoder.getPosition(), ArmInternalEncoder.getVelocity());
        endState = new TrapezoidProfile.State(setpoint, 0.0);
        profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
        timer.reset();
    }
    public void setTargetPosition(EArmPosition _setpoint)
    {
        if (_setpoint.Rotations != setpoint)
        {
            setpoint = _setpoint.Rotations;
            updateMotionprofile();
        }
    }

    public void runAutomatic()
    {
        double elaspedTime = timer.get();
        if (profile.isFinished(elaspedTime)) {
            targetState = new TrapezoidProfile.State(setpoint, 0.0);
        }
        else
        {
            targetState = profile.calculate(elaspedTime, startState, endState);
        }

        feedforward = GravityFF();
        ArmPID.setReference(targetState.position, CANSparkBase.ControlType.kPosition, 0, feedforward);

    }

    private CANSparkFlex CreateMotor(int motorId) {
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
        pid.setOutputRange(-0.05, 0.05);

        pid.setSmartMotionMaxVelocity(2, 0);
        pid.setSmartMotionMaxAccel(1, 0);
        pid.setSmartMotionMinOutputVelocity(0, 0);
        pid.setSmartMotionAllowedClosedLoopError(0.0021, 0);


        return pid;
    }


    private double GravityFF()
    {
        double cosScalar = FF * Math.cos(ArmInternalEncoder.getPosition());
        double gFF = FF * cosScalar;
        return gFF;
    }

    public Command Command_SetPosition(EArmPosition position)
    {
//        return runOnce(() -> ArmPID.setReference(position.Rotations, CANSparkBase.ControlType.kPosition, 0, 0, SparkPIDController.ArbFFUnits.kPercentOut));
        return run(() ->
            {
                SmartDashboard.putNumber("Arm Target", position.Rotations);

                //ArmPID.setReference(position.Rotations, CANSparkBase.ControlType.kSmartMotion, 0, 0, SparkPIDController.ArbFFUnits.kPercentOut);
                ArmPID.setReference(position.Rotations, CANSparkBase.ControlType.kPosition, 0, 0, SparkPIDController.ArbFFUnits.kPercentOut);
            })
            .until(() ->
            {
//                double actualRotation = ArmInternalEncoder.getPosition();
//                SmartDashboard.putNumber("Arm Rotation", actualRotation);
                return MathUtil.isNear(position.Rotations, ArmInternalEncoder.getPosition(), 0.1);
//                return Math.abs(actualRotation - position.Rotations) < 0.10; // TODO: Tune this error threshold
            });
    }

    
}