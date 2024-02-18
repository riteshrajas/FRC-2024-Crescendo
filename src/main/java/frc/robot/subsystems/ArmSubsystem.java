package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase
{
    private static final boolean EnableRightMotor = false;
    private static final boolean TunePID = false;
    private static final double ArmTolerance = 10.0 / 360.0;

    public enum EArmPosition {
        stowed(0),
        shoot_subwoofer(1),
        shoot_podium(0), //TODO: tune when added
        shoot_wing(2), //TODO: tune when added
        climb_firstpos(10),
        climb_secondpos(0),
        amp(2),
        trap(5);

        private final int Rotations;

        EArmPosition(int rotations) {
            Rotations = rotations;
        }
    }

    // -- Motors
    private TalonFX LeftMotor;
    private TalonFX RightMotor;

    // -- Configs
    private TalonFXConfiguration TalonConfigs;
    private Slot0Configs TalonConfigs_Slot0;
    private MotionMagicConfigs TalonConfigs_MotionMagic;

    // -- Control Requests
    private final MotionMagicVoltage MotionMagicRequest = new MotionMagicVoltage(0);


    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
    }

    public ArmSubsystem()
    {
        InitializeConfigs();

        LeftMotor = CreateMotor(Constants.CanivoreBusIDs.ArmLeft.GetID());

        if (EnableRightMotor)
        {
            RightMotor = CreateMotor(Constants.CanivoreBusIDs.ArmLeft.GetID());
            RightMotor.setControl(new Follower(Constants.CanivoreBusIDs.ArmLeft.GetID(), true));
        }

        ApplyConfigs();
        PublishConfigs();
    }

    private void InitializeConfigs()
    {
        TalonConfigs = new TalonFXConfiguration();
        TalonConfigs_Slot0 = TalonConfigs.Slot0;
        TalonConfigs_MotionMagic = TalonConfigs.MotionMagic;

        TalonConfigs_Slot0.kP = 10;
        TalonConfigs_Slot0.kI = 0;
        TalonConfigs_Slot0.kD = 0.2;

        TalonConfigs_Slot0.kS = 0.35; // Static
        TalonConfigs_Slot0.kA = 0.01; // Acceleration
        TalonConfigs_Slot0.kV = 0.12; // Velocity
        TalonConfigs_Slot0.kG = 0; // Gravity

        TalonConfigs_Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

        TalonConfigs_MotionMagic.MotionMagicAcceleration = 60; // rps/s acceleration (0.5 seconds)
        TalonConfigs_MotionMagic.MotionMagicCruiseVelocity = 80; // rps cruise velocity
        TalonConfigs_MotionMagic.MotionMagicJerk = 1600; // rps/s^2 jerk (0.1 seconds)
        TalonConfigs_MotionMagic.MotionMagicExpo_kA = 0.1;
        TalonConfigs_MotionMagic.MotionMagicExpo_kV = 0.1;

        MotionMagicRequest.Slot = 0;
    }

    private TalonFX CreateMotor(int deviceID)
    {
        var motor = new TalonFX(deviceID, "Canivore");
        motor.setPosition(0);
        return motor;
    }

    private void ApplyConfigs()
    {
        var timeout = 0.05;

        LeftMotor.getConfigurator().apply(TalonConfigs, timeout);
        if (EnableRightMotor)
        {
            RightMotor.getConfigurator().apply(TalonConfigs, timeout);
        }

        System.out.println("Configs Applied!");
    }

    public Command Command_SetPosition(EArmPosition position) {
        return
            run(() ->
            {
                SmartDashboard.putNumber("Arm.Target", position.Rotations);
                LeftMotor.setControl(MotionMagicRequest.withPosition(position.Rotations));
            })
            .until(() ->
            {
                double actualRotation = LeftMotor.getPosition().getValue();
                return MathUtil.isNear(position.Rotations, actualRotation, ArmTolerance);
            })
            .andThen(() -> System.out.println("Arm reached target!"));
    }

    public Command zeroArmEncoder()
    {
       return runOnce(() -> LeftMotor.setPosition(0));
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm.Position", LeftMotor.getPosition().getValue());
        UpdateConfigs();
    }

    private void PublishConfigs()
    {
        if (!TunePID) { return; }

        SmartDashboard.putNumber("Arm.Target", 0);
        SmartDashboard.putNumber("Arm.Position", 0);
        SmartDashboard.putNumber("Arm.Error", 0);

        SmartDashboard.putNumber("Arm.Configs.P", TalonConfigs_Slot0.kP);
        SmartDashboard.putNumber("Arm.Configs.I", TalonConfigs_Slot0.kI);
        SmartDashboard.putNumber("Arm.Configs.D", TalonConfigs_Slot0.kD);

        SmartDashboard.putNumber("Arm.Configs.S", TalonConfigs_Slot0.kS);
        SmartDashboard.putNumber("Arm.Configs.A", TalonConfigs_Slot0.kA);
        SmartDashboard.putNumber("Arm.Configs.V", TalonConfigs_Slot0.kV);
        SmartDashboard.putNumber("Arm.Configs.G", TalonConfigs_Slot0.kG);

        SmartDashboard.putNumber("Arm.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
        SmartDashboard.putNumber("Arm.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
        SmartDashboard.putNumber("Arm.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
        SmartDashboard.putNumber("Arm.Configs.MMExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
        SmartDashboard.putNumber("Arm.Configs.MMExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);
    }

    private void UpdateConfigs()
    {
        if (!TunePID) { return; }

        var dirty = false;

        var p = SmartDashboard.getNumber("Arm.Configs.P", TalonConfigs_Slot0.kP);
        var i = SmartDashboard.getNumber("Arm.Configs.I", TalonConfigs_Slot0.kI);
        var d = SmartDashboard.getNumber("Arm.Configs.D", TalonConfigs_Slot0.kD);

        var s = SmartDashboard.getNumber("Arm.Configs.S", TalonConfigs_Slot0.kS);
        var a = SmartDashboard.getNumber("Arm.Configs.A", TalonConfigs_Slot0.kA);
        var v = SmartDashboard.getNumber("Arm.Configs.V", TalonConfigs_Slot0.kV);
        var g = SmartDashboard.getNumber("Arm.Configs.G", TalonConfigs_Slot0.kG);

        var mmA = SmartDashboard.getNumber("Arm.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
        var mmC = SmartDashboard.getNumber("Arm.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
        var mmJ = SmartDashboard.getNumber("Arm.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
        var mmEA = SmartDashboard.getNumber("Arm.Configs.mmExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
        var mmEV = SmartDashboard.getNumber("Arm.Configs.mmExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);

        if (p != TalonConfigs_Slot0.kP) { TalonConfigs_Slot0.kP = p; dirty = true; }
        if (i != TalonConfigs_Slot0.kI) { TalonConfigs_Slot0.kI = i; dirty = true; }
        if (d != TalonConfigs_Slot0.kD) { TalonConfigs_Slot0.kD = d; dirty = true; }

        if (s != TalonConfigs_Slot0.kS) { TalonConfigs_Slot0.kS = s; dirty = true; }
        if (a != TalonConfigs_Slot0.kA) { TalonConfigs_Slot0.kA = a; dirty = true; }
        if (v != TalonConfigs_Slot0.kV) { TalonConfigs_Slot0.kV = v; dirty = true; }
        if (g != TalonConfigs_Slot0.kG) { TalonConfigs_Slot0.kG = g; dirty = true; }

        if (mmC != TalonConfigs_MotionMagic.MotionMagicCruiseVelocity) { TalonConfigs_MotionMagic.MotionMagicCruiseVelocity = mmC; dirty = true; }
        if (mmA != TalonConfigs_MotionMagic.MotionMagicAcceleration) { TalonConfigs_MotionMagic.MotionMagicAcceleration = mmA; dirty = true; }
        if (mmJ != TalonConfigs_MotionMagic.MotionMagicJerk) { TalonConfigs_MotionMagic.MotionMagicJerk = mmJ; dirty = true; }
        if (mmEA != TalonConfigs_MotionMagic.MotionMagicExpo_kA) { TalonConfigs_MotionMagic.MotionMagicExpo_kA = mmEA; dirty = true; }
        if (mmEV != TalonConfigs_MotionMagic.MotionMagicExpo_kV) { TalonConfigs_MotionMagic.MotionMagicExpo_kV = mmEV; dirty = true; }

        if (dirty)
        {
            ApplyConfigs();
        }
    }
}