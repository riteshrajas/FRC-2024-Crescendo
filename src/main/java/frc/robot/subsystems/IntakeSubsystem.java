package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;
import frc.robot.RobotContainer;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSubsystem extends SubsystemBase {
    public enum EPivotPosition {
        Stowed(0),
        Intake(-18),
        Shoot_speaker(-7.75),
        Amp(-32.7),
        Trap(0);

        private final double Rotations;

        EPivotPosition(double rotations) {
            Rotations = rotations;
        }
    }

    public enum EOutakeType {
        amp(-20),
        speaker(-60),

        trap(0);

        private final double RPS;

        EOutakeType(double rps) {
            RPS = rps;
        }
    }

    private TalonFXConfiguration TuneConfigs = null;
    private final double PivotTolerance = 15.0 / 360.0;
    private final double TEMP_IntakeVoltage = 5;


    private TalonFX PivotMotor;
    private TalonFX IntakeMotor;
    private TalonFXConfiguration PivotConfigs;
    private TalonFXConfiguration IntakeConfigs;

    private final MotionMagicVoltage MotionMagicRequest = new MotionMagicVoltage(0);

    private final VelocityTorqueCurrentFOC IntakeRequest = new VelocityTorqueCurrentFOC(0);

    private final VoltageOut VoltageRequest = new VoltageOut(0).withEnableFOC(true);

    private final DutyCycleOut DutyCycleRequest = new DutyCycleOut(0);

    double lastCurrent = 0;
    int currentSpikeCount = 0;

    public IntakeSubsystem() {
        PivotConfigs = CreateConfigs(true);
        IntakeConfigs = CreateConfigs(false);

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

    private TalonFXConfiguration CreateConfigs(boolean isPivot) {
        var configs = new TalonFXConfiguration();
        var slotConfigs = configs.Slot0;
        var motionMagicConfigs = configs.MotionMagic;

        if (isPivot)
        {
            slotConfigs.kP = 10;
            slotConfigs.kI = 0;
            slotConfigs.kD = 0.2;

            slotConfigs.kS = 0.35; // Static
            slotConfigs.kA = 0.01; // Acceleration
            slotConfigs.kV = 0.12; // Velocity
            slotConfigs.kG = 0; // Gravity

            slotConfigs.withGravityType(GravityTypeValue.Arm_Cosine);

            motionMagicConfigs.MotionMagicAcceleration = 300; // rps/s acceleration (0.5 seconds)
            motionMagicConfigs.MotionMagicCruiseVelocity = 160; // rps cruise velocity
            motionMagicConfigs.MotionMagicJerk = 1600; // rps/s^2 jerk (0.1 seconds)
            motionMagicConfigs.MotionMagicExpo_kA = 0.1;
            motionMagicConfigs.MotionMagicExpo_kV = 0.1;

            MotionMagicRequest.Slot = 0;
        }
        else
        {
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            slotConfigs.kP = 10;
            slotConfigs.kI = 0;
            slotConfigs.kD = 0;

            slotConfigs.kS = 40; // Static
            slotConfigs.kA = 0; // Acceleration
            slotConfigs.kV = 0.25; // Velocity
            slotConfigs.kG = 0; // Gravity

            motionMagicConfigs.MotionMagicAcceleration = 0; // rps/s acceleration (0.5 seconds)
            motionMagicConfigs.MotionMagicCruiseVelocity = 0; // rps cruise velocity
            motionMagicConfigs.MotionMagicJerk = 0; // rps/s^2 jerk (0.1 seconds)
            motionMagicConfigs.MotionMagicExpo_kA = 0;
            motionMagicConfigs.MotionMagicExpo_kV = 0;

            MotionMagicRequest.Slot = 0;
        }
        return configs;
    }


    private TalonFX CreateMotor(int deviceID) {
        var motor = new TalonFX(deviceID, Constants.CanivoreBusIDs.BusName);
        motor.setPosition(0);
        return motor;
    }

    private void ApplyConfigs() {
        var timeout = 0.05;

        PivotMotor.getConfigurator().apply(PivotConfigs, timeout);
        IntakeMotor.getConfigurator().apply(IntakeConfigs, timeout);

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
                 IntakeMotor.setControl(VoltageRequest.withOutput(TEMP_IntakeVoltage));
                 //IntakeMotor.setControl(IntakeRequest.withVelocity(35));
             });
    }

    public Command Command_IntakeNote()
    {


        return
            startEnd
            (
                () ->
                {
                    currentSpikeCount = 0;
                    lastCurrent = IntakeMotor.getStatorCurrent().getValue();

                    //SmartDashboard.putNumber("Intake.TargetVelocity", 2000);
                    IntakeMotor.setControl(VoltageRequest.withOutput(TEMP_IntakeVoltage));
                    //IntakeMotor.setControl(IntakeRequest.withVelocity(35));
                },
                () -> IntakeMotor.stopMotor()
            )
            .until(() ->
            {


                double curCurrent = IntakeMotor.getStatorCurrent().getValue();
                SmartDashboard.putNumber("Intake.CurrentDelta", curCurrent - lastCurrent);
                if (curCurrent - lastCurrent > 25)
                {
                    CommandScheduler.getInstance().schedule(
                        Commands.startEnd(
                                () -> RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1),
                                () -> RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)
                        ).withTimeout(0.5));


                    currentSpikeCount++;
                }

                lastCurrent = curCurrent;
                return currentSpikeCount >= 1;
            });
    }

    public Command Command_MoveNote(boolean forward)
    {
        return startEnd
            (() -> IntakeMotor.setControl(IntakeRequest.withVelocity(forward ? -3 : 3)),
            () -> IntakeMotor.stopMotor()
        );
    }


    public Command Command_Outtake(EOutakeType outtakeType)
    {
        return startEnd(
                () ->
                {
                    SmartDashboard.putNumber("Intake.TargetVelocity", outtakeType.RPS);
                    //IntakeMotor.setControl(DutyCycleRequest.withOutput(-0.75));
                    IntakeMotor.setControl(IntakeRequest.withVelocity(outtakeType.RPS));
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
        SmartDashboard.putNumber("Intake.CurrentSpikeCount", currentSpikeCount);
        SmartDashboard.putNumber("Intake.PivotPosition", PivotMotor.getPosition().getValue());
        //SmartDashboard.putNumber("Intake.ActualVelocity", PivotMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Intake.ActualCurrent", IntakeMotor.getStatorCurrent().getValue());

        UpdateConfigs();
    }

    private void PublishConfigs()
    {
        if (TuneConfigs == null) { return; }

        SmartDashboard.putNumber("Pivot.Target", 0);
        SmartDashboard.putNumber("Pivot.Position", 0);
        SmartDashboard.putNumber("Pivot.Error", 0);

        SmartDashboard.putNumber("Pivot.Configs.P", TuneConfigs.Slot0.kP);
        SmartDashboard.putNumber("Pivot.Configs.I", TuneConfigs.Slot0.kI);
        SmartDashboard.putNumber("Pivot.Configs.D", TuneConfigs.Slot0.kD);

        SmartDashboard.putNumber("Pivot.Configs.S", TuneConfigs.Slot0.kS);
        SmartDashboard.putNumber("Pivot.Configs.A", TuneConfigs.Slot0.kA);
        SmartDashboard.putNumber("Pivot.Configs.V", TuneConfigs.Slot0.kV);
        SmartDashboard.putNumber("Pivot.Configs.G", TuneConfigs.Slot0.kG);

        SmartDashboard.putNumber("Pivot.Configs.MMCruise", TuneConfigs.MotionMagic.MotionMagicCruiseVelocity);
        SmartDashboard.putNumber("Pivot.Configs.MMAccel", TuneConfigs.MotionMagic.MotionMagicAcceleration);
        SmartDashboard.putNumber("Pivot.Configs.MMJerk", TuneConfigs.MotionMagic.MotionMagicJerk);
        SmartDashboard.putNumber("Pivot.Configs.MMExpoA", TuneConfigs.MotionMagic.MotionMagicExpo_kA);
        SmartDashboard.putNumber("Pivot.Configs.MMExpoV", TuneConfigs.MotionMagic.MotionMagicExpo_kV);
    }

    private void UpdateConfigs()
    {
        if (TuneConfigs == null) { return; }

        var dirty = false;
//
        var p = SmartDashboard.getNumber("Pivot.Configs.P", TuneConfigs.Slot0.kP);
        var i = SmartDashboard.getNumber("Pivot.Configs.I", TuneConfigs.Slot0.kI);
        var d = SmartDashboard.getNumber("Pivot.Configs.D", TuneConfigs.Slot0.kD);

        var s = SmartDashboard.getNumber("Pivot.Configs.S", TuneConfigs.Slot0.kS);
        var a = SmartDashboard.getNumber("Pivot.Configs.A", TuneConfigs.Slot0.kA);
        var v = SmartDashboard.getNumber("Pivot.Configs.V", TuneConfigs.Slot0.kV);
        var g = SmartDashboard.getNumber("Pivot.Configs.G", TuneConfigs.Slot0.kG);

        var mmA = SmartDashboard.getNumber("Pivot.Configs.MMAccel", TuneConfigs.MotionMagic.MotionMagicAcceleration);
        var mmC = SmartDashboard.getNumber("Pivot.Configs.MMCruise", TuneConfigs.MotionMagic.MotionMagicCruiseVelocity);
        var mmJ = SmartDashboard.getNumber("Pivot.Configs.MMJerk", TuneConfigs.MotionMagic.MotionMagicJerk);
        var mmEA = SmartDashboard.getNumber("Pivot.Configs.mmExpoA", TuneConfigs.MotionMagic.MotionMagicExpo_kA);
        var mmEV = SmartDashboard.getNumber("Pivot.Configs.mmExpoV", TuneConfigs.MotionMagic.MotionMagicExpo_kV);



        if (p != TuneConfigs.Slot0.kP) { TuneConfigs.Slot0.kP = p; dirty = true; }
        if (i != TuneConfigs.Slot0.kI) { TuneConfigs.Slot0.kI = i; dirty = true; }
        if (d != TuneConfigs.Slot0.kD) { TuneConfigs.Slot0.kD = d; dirty = true; }

        if (s != TuneConfigs.Slot0.kS) { TuneConfigs.Slot0.kS = s; dirty = true; }
        if (a != TuneConfigs.Slot0.kA) { TuneConfigs.Slot0.kA = a; dirty = true; }
        if (v != TuneConfigs.Slot0.kV) { TuneConfigs.Slot0.kV = v; dirty = true; }
        if (g != TuneConfigs.Slot0.kG) { TuneConfigs.Slot0.kG = g; dirty = true; }

        if (mmC != TuneConfigs.MotionMagic.MotionMagicCruiseVelocity) { TuneConfigs.MotionMagic.MotionMagicCruiseVelocity = mmC; dirty = true; }
        if (mmA != TuneConfigs.MotionMagic.MotionMagicAcceleration) { TuneConfigs.MotionMagic.MotionMagicAcceleration = mmA; dirty = true; }
        if (mmJ != TuneConfigs.MotionMagic.MotionMagicJerk) { TuneConfigs.MotionMagic.MotionMagicJerk = mmJ; dirty = true; }
        if (mmEA != TuneConfigs.MotionMagic.MotionMagicExpo_kA) { TuneConfigs.MotionMagic.MotionMagicExpo_kA = mmEA; dirty = true; }
        if (mmEV != TuneConfigs.MotionMagic.MotionMagicExpo_kV) { TuneConfigs.MotionMagic.MotionMagicExpo_kV = mmEV; dirty = true; }


        if (dirty)
        {
            ApplyConfigs();
        }
    }
}

