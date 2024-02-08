package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase
{
    public enum EArmPosition
    {
        stowed              (0),
        shoot_podium        (1),
        shoot_wing          (-1),
        climb_firstpos      (10),
        climb_secondpos     (0),
        amp                 (0),
        trap                (0);

        private final int Rotations;

        EArmPosition(int rotations)
        {
            Rotations = rotations;
        }
    }

    private CANSparkFlex ArmMotorLeft;
    private CANSparkFlex ArmMotorRight;

    private SparkPIDController ArmPID;
    private RelativeEncoder ArmInternalEncoder;
    private RelativeEncoder ArmExternalEncoder; // Through Bore Encoder

    public ArmSubsystem()
    {
        ArmMotorLeft = CreateMotor(Constants.Arm.ARM_MOTOR_LEFT);
//        ArmMotorRight = CreateMotor(Constants.Arm.ARM_MOTOR_RIGHT);
//        ArmMotorRight.follow(ArmMotorLeft);

        ArmPID = CreatePID(ArmMotorLeft);
        ArmInternalEncoder = ArmMotorLeft.getEncoder();
        ArmExternalEncoder = ArmMotorLeft.getExternalEncoder(8192);

        // Uncomment this out when we have the Through Bore Encoder connected (and redo positions)
        // Also switch any other uses of ArmInternalEncoder to ArmExternalEncoder
        // ArmPID.setFeedbackDevice(ArmExternalEncoder);
    }

    private CANSparkFlex CreateMotor(int motorId)
    {
        CANSparkFlex motor = new CANSparkFlex(motorId, CANSparkLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.burnFlash();

        return motor;
    }

    private SparkPIDController CreatePID(CANSparkFlex motor)
    {
        var pid = motor.getPIDController();

        pid.setP(0.1);
        pid.setI(1e-4);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF(0);
        pid.setOutputRange(-1, 1);

        return pid;
    }

    public Command Command_SetPosition(EArmPosition position)
    {
        return run(() ->
            {
                ArmPID.setReference(position.Rotations, CANSparkBase.ControlType.kPosition);
            })
            .until(() ->
            {
                double actualRotation = ArmInternalEncoder.getPosition();

                SmartDashboard.putNumber("Arm Rotation", actualRotation);
                SmartDashboard.putNumber("Arm Target", position.Rotations);

                return Math.abs(actualRotation - position.Rotations) < 0.01; // TODO: Tune this error threshold
            });
    }

    
}