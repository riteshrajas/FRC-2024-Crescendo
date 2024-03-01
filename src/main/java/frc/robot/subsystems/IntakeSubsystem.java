package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSubsystem extends SubsystemBase {
    public enum EPivotPosition {
        Stowed(0),
        Intake(-18.75),
        Shoot_speaker(-7.75),
        Amp(-33),
        Trap(0);

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
    private static final double TEMP_IntakeVoltage = 7;


    private TalonFX PivotMotor;
    private TalonFX IntakeMotor;
    private TalonFXConfiguration TalonConfigs;
    private Slot0Configs TalonConfigs_Slot0;

    private MotionMagicConfigs TalonConfigs_MotionMagic;


    private final MotionMagicVoltage MotionMagicRequest = new MotionMagicVoltage(0);

    private final VoltageOut VoltageRequest = new VoltageOut(0);

    private final DutyCycleOut DutyCycleRequest = new DutyCycleOut(0);

    public IntakeSubsystem() {
        InitializeConfigs();
        PivotMotor = CreateMotor(Constants.CanivoreBusIDs.IntakePivot.GetID());
        IntakeMotor = CreateMotor(Constants.CanivoreBusIDs.IntakeMotor.GetID());

        ApplyConfigs();
        PublishConfigs();

        PivotMotor.setControl(MotionMagicRequest.withPosition(EPivotPosition.Stowed.Rotations));
    }

    public double GetPivotPos()
    {
        return PivotMotor.getPosition().getValue();
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

        TalonConfigs_MotionMagic.MotionMagicAcceleration = 300; // rps/s acceleration (0.5 seconds)
        TalonConfigs_MotionMagic.MotionMagicCruiseVelocity = 160; // rps cruise velocity
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
        IntakeMotor.getConfigurator().apply(TalonConfigs, timeout);

        System.out.println("Configs Applied!");
    }

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
        return runOnce( () ->
             {
                 IntakeMotor.setControl(VoltageRequest.withOutput(TEMP_IntakeVoltage).withEnableFOC(true));
             });
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
                    lastCurrent.set(IntakeMotor.getStatorCurrent().getValue());

                    //SmartDashboard.putNumber("Intake.TargetVelocity", 2000);
                    IntakeMotor.setControl(VoltageRequest.withOutput(TEMP_IntakeVoltage).withEnableFOC(true));
                },
                () -> IntakeMotor.stopMotor()
            )
            .until(() ->
            {
                SmartDashboard.putNumber("Intake.CurrentSpikeCount", currentSpikeCount.get());

                double curCurrent = IntakeMotor.getStatorCurrent().getValue();
//                double curCurrent = IntakeMotor.getOutputCurrent();
                if (curCurrent - lastCurrent.get() > 25)
                {
                    currentSpikeCount.getAndIncrement();
                }

                lastCurrent.set(curCurrent);
                return currentSpikeCount.get() >= 1;
            });
    }


    public Command Command_Outtake(EOutakeType outtakeType)
    {
        return startEnd(
                () ->
                {
                    //SmartDashboard.putNumber("Intake.TargetVelocity", outtakeType.RPM);
                    IntakeMotor.setControl(DutyCycleRequest.withOutput(-0.75));
                },
                () -> IntakeMotor.stopMotor()
        );
    }


    public Command Command_StopIntake() //use this in auto just in case we miss a note
    {
        return runOnce(() -> IntakeMotor.stopMotor());
    }


    public Command Command_ZeroPivotEncoder()
    {
        return runOnce(() -> PivotMotor.setPosition(0))
                .ignoringDisable(true);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Intake.PivotPosition", PivotMotor.getPosition().getValue());
        //SmartDashboard.putNumber("Intake.ActualVelocity", PivotMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Intake.ActualCurrent", IntakeMotor.getStatorCurrent().getValue());

        UpdateConfigs();
    }

    private void PublishConfigs()
    {
        if (!TunePID) { return; }

//        SmartDashboard.putNumber("Intake.Configs.P", IntakePID.getP());
//        SmartDashboard.putNumber("Intake.Configs.I", IntakePID.getI());
//        SmartDashboard.putNumber("Intake.Configs.D", IntakePID.getD());

        SmartDashboard.putNumber("Pivot.Target", 0);
        SmartDashboard.putNumber("Pivot.Position", 0);
        SmartDashboard.putNumber("Pivot.Error", 0);

        SmartDashboard.putNumber("Pivot.Configs.P", TalonConfigs_Slot0.kP);
        SmartDashboard.putNumber("Pivot.Configs.I", TalonConfigs_Slot0.kI);
        SmartDashboard.putNumber("Pivot.Configs.D", TalonConfigs_Slot0.kD);

        SmartDashboard.putNumber("Pivot.Configs.S", TalonConfigs_Slot0.kS);
        SmartDashboard.putNumber("Pivot.Configs.A", TalonConfigs_Slot0.kA);
        SmartDashboard.putNumber("Pivot.Configs.V", TalonConfigs_Slot0.kV);
        SmartDashboard.putNumber("Pivot.Configs.G", TalonConfigs_Slot0.kG);

        SmartDashboard.putNumber("Pivot.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
        SmartDashboard.putNumber("Pivot.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
        SmartDashboard.putNumber("Pivot.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
        SmartDashboard.putNumber("Pivot.Configs.MMExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
        SmartDashboard.putNumber("Pivot.Configs.MMExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);
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
        var pivotDirty = false;
//
        var p = SmartDashboard.getNumber("Pivot.Configs.P", TalonConfigs_Slot0.kP);
        var i = SmartDashboard.getNumber("Pivot.Configs.I", TalonConfigs_Slot0.kI);
        var d = SmartDashboard.getNumber("Pivot.Configs.D", TalonConfigs_Slot0.kD);

        var s = SmartDashboard.getNumber("Pivot.Configs.S", TalonConfigs_Slot0.kS);
        var a = SmartDashboard.getNumber("Pivot.Configs.A", TalonConfigs_Slot0.kA);
        var v = SmartDashboard.getNumber("Pivot.Configs.V", TalonConfigs_Slot0.kV);
        var g = SmartDashboard.getNumber("Pivot.Configs.G", TalonConfigs_Slot0.kG);

        var mmA = SmartDashboard.getNumber("Pivot.Configs.MMAccel", TalonConfigs_MotionMagic.MotionMagicAcceleration);
        var mmC = SmartDashboard.getNumber("Pivot.Configs.MMCruise", TalonConfigs_MotionMagic.MotionMagicCruiseVelocity);
        var mmJ = SmartDashboard.getNumber("Pivot.Configs.MMJerk", TalonConfigs_MotionMagic.MotionMagicJerk);
        var mmEA = SmartDashboard.getNumber("Pivot.Configs.mmExpoA", TalonConfigs_MotionMagic.MotionMagicExpo_kA);
        var mmEV = SmartDashboard.getNumber("Pivot.Configs.mmExpoV", TalonConfigs_MotionMagic.MotionMagicExpo_kV);



        if (p != TalonConfigs_Slot0.kP) { TalonConfigs_Slot0.kP = p; pivotDirty = true; }
        if (i != TalonConfigs_Slot0.kI) { TalonConfigs_Slot0.kI = i; pivotDirty = true; }
        if (d != TalonConfigs_Slot0.kD) { TalonConfigs_Slot0.kD = d; pivotDirty = true; }

        if (s != TalonConfigs_Slot0.kS) { TalonConfigs_Slot0.kS = s; pivotDirty = true; }
        if (a != TalonConfigs_Slot0.kA) { TalonConfigs_Slot0.kA = a; pivotDirty = true; }
        if (v != TalonConfigs_Slot0.kV) { TalonConfigs_Slot0.kV = v; pivotDirty = true; }
        if (g != TalonConfigs_Slot0.kG) { TalonConfigs_Slot0.kG = g; pivotDirty = true; }

        if (mmC != TalonConfigs_MotionMagic.MotionMagicCruiseVelocity) { TalonConfigs_MotionMagic.MotionMagicCruiseVelocity = mmC; pivotDirty = true; }
        if (mmA != TalonConfigs_MotionMagic.MotionMagicAcceleration) { TalonConfigs_MotionMagic.MotionMagicAcceleration = mmA; pivotDirty = true; }
        if (mmJ != TalonConfigs_MotionMagic.MotionMagicJerk) { TalonConfigs_MotionMagic.MotionMagicJerk = mmJ; pivotDirty = true; }
        if (mmEA != TalonConfigs_MotionMagic.MotionMagicExpo_kA) { TalonConfigs_MotionMagic.MotionMagicExpo_kA = mmEA; pivotDirty = true; }
        if (mmEV != TalonConfigs_MotionMagic.MotionMagicExpo_kV) { TalonConfigs_MotionMagic.MotionMagicExpo_kV = mmEV; pivotDirty = true; }


        if (pivotDirty)
        {
            ApplyConfigs();
        }
    }
}

