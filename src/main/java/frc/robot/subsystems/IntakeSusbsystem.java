package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSusbsystem extends SubsystemBase {
    public enum EPivotPosition {
        Stowed(0),
        Intake(19.27),
        Shoot_speaker(8.85),
        amp(33),
        trap(0);

        private final double Rotations;

        EPivotPosition(double rotations) {
            Rotations = rotations;
        }
    }

    public enum EOutakeType {
        amp(3000),
        speaker(0),
        trap(0);

        private final double RPM;

        EOutakeType(double rpm) {
            RPM = rpm;
        }
    }

    private static final boolean TunePID = false;
    private static final double PivotTolerance = 15.0 / 360.0;
    private static final double TEMP_IntakeVoltage = -7;

    private Timer rumbleTimer = new Timer();
    private Timer autoOuttakeTimer = new Timer();

    private TalonFX PivotMotor;

    private SparkPIDController IntakePID;
    private CANSparkFlex IntakeMotor;
    private RelativeEncoder IntakeEncoder;
//    private TalonFX IntakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);

    private TalonFXConfiguration TalonConfigs;
    private Slot0Configs TalonConfigs_Slot0;
    private MotionMagicConfigs TalonConfigs_MotionMagic;

    private DigitalInput VexLimitSwitch;


    private final MotionMagicVoltage MotionMagicRequest = new MotionMagicVoltage(0);

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

//    private final VelocityDutyCycle Velocity = new VelocityDutyCycle(0);

    public IntakeSusbsystem() {
        InitializeConfigs();
        PivotMotor = CreateMotor(Constants.CanivoreBusIDs.IntakePivot.GetID());

        IntakeMotor = CreateSparkMotor(Constants.RioCanBusIDs.IntakeMotor.ordinal());
        IntakePID = CreatePID(IntakeMotor);
        IntakeEncoder = IntakeMotor.getEncoder();
        IntakeMotor.burnFlash();
        VexLimitSwitch = new DigitalInput(0);

        ApplyConfigs();
        PublishConfigs();

        PivotMotor.setControl(MotionMagicRequest.withPosition(EPivotPosition.Stowed.Rotations));
    }

    private void InitializeConfigs() {
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

    private TalonFX CreateMotor(int deviceID) {
        var motor = new TalonFX(deviceID, Constants.CanivoreBusIDs.BusName);
        motor.setPosition(0);
        return motor;
    }

    private void ApplyConfigs() {
        var timeout = 0.05;

        PivotMotor.getConfigurator().apply(TalonConfigs, timeout);

        System.out.println("Configs Applied!");
    }

    private CANSparkFlex CreateSparkMotor(int motorId) {
        CANSparkFlex motor = new CANSparkFlex(motorId, CANSparkLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setInverted(true);

        return motor;
    }

    private SparkPIDController CreatePID(CANSparkFlex motor) {
        var pid = motor.getPIDController();

        pid.setP(0.0007);
        pid.setI(0.0000000);
        pid.setD(0.023);
        pid.setIZone(0);
        pid.setFF(0.000015);
        pid.setOutputRange(-1, 1);

//        pid.setSmartMotionMaxVelocity(1000, 0);
//        pid.setSmartMotionMaxAccel(500, 0);
//        pid.setSmartMotionMinOutputVelocity(0, 0);
//        pid.setSmartMotionAllowedClosedLoopError(0.0021, 0);


        return pid;
    }

    /*
    Command for setting the pivot of the intake using motion magic foc
    */
    public Command Command_SetPivotPosition(EPivotPosition position) { //Controls the pivot of the intake
        return run(() ->
        {
            SmartDashboard.putNumber("Intake.PivotTarget", position.Rotations);

            PivotMotor.setControl(MotionMagicRequest.withPosition(position.Rotations));
        })
        .until(() ->
        {
            double actualRotation = PivotMotor.getPosition().getValue();
            return MathUtil.isNear(position.Rotations, actualRotation, PivotTolerance);
        })
        .andThen(() -> System.out.println("Pivot has reached it's target!"));
    }


    public Command Command_PreIntakeSpinUp()
    {
        // TODO: This should be a startEnd command that stops the motor when it finishes in case the command gets interrupted,
        //       but we can't do that right now due to how the intake and arm subsystems are divided up. Until then we'll just
        //       spin up the wheels and rely on the invoking command sequence to stop the intake.
        return
            runOnce( () -> IntakePID.setReference(TEMP_IntakeVoltage, CANSparkBase.ControlType.kVoltage) );
    }

    public Command Command_IntakeNote()
    {
        AtomicReference<Double> lastCurrent = new AtomicReference<>((double) 0);
        AtomicInteger currentSpikeCount = new AtomicInteger();

        return
            startEnd
            (
                () ->
                {
                    currentSpikeCount.set(0);
                    lastCurrent.set(IntakeMotor.getOutputCurrent());

                    //SmartDashboard.putNumber("Intake.TargetVelocity", 2000);
                    //IntakePID.setReference(2000, CANSparkBase.ControlType.kVelocity);
                    IntakePID.setReference(TEMP_IntakeVoltage, CANSparkBase.ControlType.kVoltage);
                },
                () -> IntakeMotor.stopMotor()
            )
            .until(() ->
            {
                SmartDashboard.putNumber("Intake.CurrentSpikeCount", currentSpikeCount.get());

                double curCurrent = IntakeMotor.getOutputCurrent();
                if (curCurrent - lastCurrent.get() > 20)
                {
                    currentSpikeCount.getAndIncrement();
                }

                lastCurrent.set(curCurrent);
                return currentSpikeCount.get() >= 2;
            });
    }


    public Command Command_Outtake(EOutakeType outtakeType)
    {
        return startEnd(
                () ->
                {
                    //SmartDashboard.putNumber("Intake.TargetVelocity", outtakeType.RPM);
                    //IntakePID.setReference(outtakeType.RPM, CANSparkBase.ControlType.kVelocity);
                    IntakePID.setReference(12, CANSparkBase.ControlType.kVoltage);
                },
                () -> IntakeMotor.stopMotor()
        );
    }


    public Command Command_StopIntake() //use this in auto just incase we miss a note note
    {
        return runOnce(() -> IntakeMotor.stopMotor());
    }


    /*
    Command for outtaking a note into a scoring location during auto, take in account
    elasped time and not triggers
    */
    public Command Command_OuttakeNoteAuto(EOutakeType outakeType)
    {
        return startEnd(
                () -> {
                    autoOuttakeTimer.start();
                    IntakePID.setReference(outakeType.RPM, CANSparkBase.ControlType.kVelocity);
                },
                () -> {
                    IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
                    autoOuttakeTimer.stop();
                    autoOuttakeTimer.reset();
                }
        ).until(() ->
        {
            return autoOuttakeTimer.hasElapsed(0.5);
        });
    }


    /*
    Command for zeroing the encoder of the intake pivot
    */
    public Command zeroPivotEncoder()
    {
        return runOnce(() -> PivotMotor.setPosition(0));
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Intake.PivotPosition", PivotMotor.getPosition().getValue());
        SmartDashboard.putNumber("Intake.ActualVelocity", IntakeEncoder.getVelocity());
        SmartDashboard.putNumber("Intake.ActualCurrent", IntakeMotor.getOutputCurrent());

        UpdateConfigs();
    }

    private void PublishConfigs()
    {
        if (!TunePID) { return; }

//        SmartDashboard.putNumber("Intake.Configs.P", IntakePID.getP());
//        SmartDashboard.putNumber("Intake.Configs.I", IntakePID.getI());
//        SmartDashboard.putNumber("Intake.Configs.D", IntakePID.getD());

//        SmartDashboard.putNumber("Pivot.Target", 0);
//        SmartDashboard.putNumber("Pivot.Position", 0);
//        SmartDashboard.putNumber("Pivot.Error", 0);
//
//        SmartDashboard.putNumber("Pivot.Configs.P", TalonConfigs_Slot0.kP);
//        SmartDashboard.putNumber("Pivot.Configs.I", TalonConfigs_Slot0.kI);
//        SmartDashboard.putNumber("Pivot.Configs.D", TalonConfigs_Slot0.kD);
//
//        SmartDashboard.putNumber("Pivot.Configs.S", TalonConfigs_Slot0.kS);
//        SmartDashboard.putNumber("Pivot.Configs.A", TalonConfigs_Slot0.kA);
//        SmartDashboard.putNumber("Pivot.Configs.V", TalonConfigs_Slot0.kV);
//        SmartDashboard.putNumber("Pivot.Configs.G", TalonConfigs_Slot0.kG);
//
//        SmartDashboard.putNumber("Pivot.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
//        SmartDashboard.putNumber("Pivot.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
//        SmartDashboard.putNumber("Pivot.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
//        SmartDashboard.putNumber("Pivot.Configs.MMExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
//        SmartDashboard.putNumber("Pivot.Configs.MMExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);
    }

    private void UpdateConfigs()
    {
        if (!TunePID) { return; }

        // -- Intake
//        var intakeDirty = false;
//        var intakeP = SmartDashboard.getNumber("Intake.Configs.P", IntakePID.getP());
//        var intakeI = SmartDashboard.getNumber("Intake.Configs.I", IntakePID.getI());
//        var intakeD = SmartDashboard.getNumber("Intake.Configs.D", IntakePID.getD());
//
//        if (intakeP != IntakePID.getP()) { IntakePID.setP(intakeP); intakeDirty = true; System.out.println("Set P"); }
//        if (intakeI != IntakePID.getI()) { IntakePID.setI(intakeI); intakeDirty = true; System.out.println("Set I"); }
//        if (intakeD != IntakePID.getD()) { IntakePID.setD(intakeD); intakeDirty = true; System.out.println("Set D"); }
//
//        if (intakeDirty)
//        {
//            IntakeMotor.burnFlash();
//        }

        // -- Pivot
//        var pivotDirty = false;
//
//        var p = SmartDashboard.getNumber("Pivot.Configs.P", TalonConfigs_Slot0.kP);
//        var i = SmartDashboard.getNumber("Pivot.Configs.I", TalonConfigs_Slot0.kI);
//        var d = SmartDashboard.getNumber("Pivot.Configs.D", TalonConfigs_Slot0.kD);
//
//        var s = SmartDashboard.getNumber("Pivot.Configs.S", TalonConfigs_Slot0.kS);
//        var a = SmartDashboard.getNumber("Pivot.Configs.A", TalonConfigs_Slot0.kA);
//        var v = SmartDashboard.getNumber("Pivot.Configs.V", TalonConfigs_Slot0.kV);
//        var g = SmartDashboard.getNumber("Pivot.Configs.G", TalonConfigs_Slot0.kG);
//
//        var mmA = SmartDashboard.getNumber("Pivot.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
//        var mmC = SmartDashboard.getNumber("Pivot.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
//        var mmJ = SmartDashboard.getNumber("Pivot.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
//        var mmEA = SmartDashboard.getNumber("Pivot.Configs.mmExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
//        var mmEV = SmartDashboard.getNumber("Pivot.Configs.mmExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);
//
//
//
//        if (p != TalonConfigs_Slot0.kP) { TalonConfigs_Slot0.kP = p; pivotDirty = true; }
//        if (i != TalonConfigs_Slot0.kI) { TalonConfigs_Slot0.kI = i; pivotDirty = true; }
//        if (d != TalonConfigs_Slot0.kD) { TalonConfigs_Slot0.kD = d; pivotDirty = true; }
//
//        if (s != TalonConfigs_Slot0.kS) { TalonConfigs_Slot0.kS = s; pivotDirty = true; }
//        if (a != TalonConfigs_Slot0.kA) { TalonConfigs_Slot0.kA = a; pivotDirty = true; }
//        if (v != TalonConfigs_Slot0.kV) { TalonConfigs_Slot0.kV = v; pivotDirty = true; }
//        if (g != TalonConfigs_Slot0.kG) { TalonConfigs_Slot0.kG = g; pivotDirty = true; }
//
//        if (mmC != TalonConfigs_MotionMagic.MotionMagicCruiseVelocity) { TalonConfigs_MotionMagic.MotionMagicCruiseVelocity = mmC; pivotDirty = true; }
//        if (mmA != TalonConfigs_MotionMagic.MotionMagicAcceleration) { TalonConfigs_MotionMagic.MotionMagicAcceleration = mmA; pivotDirty = true; }
//        if (mmJ != TalonConfigs_MotionMagic.MotionMagicJerk) { TalonConfigs_MotionMagic.MotionMagicJerk = mmJ; pivotDirty = true; }
//        if (mmEA != TalonConfigs_MotionMagic.MotionMagicExpo_kA) { TalonConfigs_MotionMagic.MotionMagicExpo_kA = mmEA; pivotDirty = true; }
//        if (mmEV != TalonConfigs_MotionMagic.MotionMagicExpo_kV) { TalonConfigs_MotionMagic.MotionMagicExpo_kV = mmEV; pivotDirty = true; }
//
//
//        if (pivotDirty)
//        {
//            ApplyConfigs();
//        }
    }
}

