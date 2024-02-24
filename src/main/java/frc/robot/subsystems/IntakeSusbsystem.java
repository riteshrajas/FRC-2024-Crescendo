package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;
import frc.robot.RobotContainer;
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

    public enum EOutakeType {
        amp(0),
        speaker(0),
        trap(0);

        private final double RPM;

        EOutakeType(double rpm) {
            RPM = rpm;
        }
    }

    private static final boolean TunePID = false;
    private static final double ArmTolerance = 10.0 / 360.0;

    private Timer rumbleTimer = new Timer();
    private Timer autoOuttakeTimer = new Timer();

    private TalonFX PivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);

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
        PivotMotor = CreateMotor(Constants.Intake.PIVOT_MOTOR_ID);
        IntakeMotor = CreateSparkMotor(23);
        IntakePID = CreatePID(IntakeMotor);
        IntakeEncoder = IntakeMotor.getEncoder();
        MotionMagicRequest.Slot = 1;
        IntakeMotor.burnFlash();
        VexLimitSwitch = new DigitalInput(0);

        ApplyConfigs();
        PublishConfigs();

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
        var motor = new TalonFX(deviceID, "Canivore");
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

        pid.setP(0.000325);
        pid.setI(0.0000001);
        pid.setD(0.01357);
        pid.setIZone(0);
        pid.setFF(0.000015);
        pid.setOutputRange(-1, 1);

//        pid.setSmartMotionMaxVelocity(1000, 0);
//        pid.setSmartMotionMaxAccel(500, 0);
//        pid.setSmartMotionMinOutputVelocity(0, 0);
//        pid.setSmartMotionAllowedClosedLoopError(0.0021, 0);


        return pid;
    }

    private boolean isPressed() {
        return VexLimitSwitch.get();
    }

    /*
    Command for setting the pivot of the intake using motion magic foc
    */
    public Command Command_SetPivotPosition(EPivotPosition position) { //Controls the pivot of the intake
        return run(() ->
        {
            SmartDashboard.putNumber("Pivot Target", position.Rotations);

            PivotMotor.setControl(MotionMagicRequest.withPosition(position.Rotations));
        })
                .until(() ->
                {
                    double actualRotation = PivotMotor.getPosition().getValue();
                    return MathUtil.isNear(position.Rotations, actualRotation, ArmTolerance);
                })
                .andThen(() -> System.out.println("Pivot has reached it's target!"));
    }

    /**
     * Command for intaking the note, will stop intake if limit switch is pressed or trigger is released
     **/
    public Command Command_IntakeNote() {
        return
                Commands.sequence(
                startEnd(
                        () -> {
                            IntakePID.setReference(2000, CANSparkBase.ControlType.kVelocity);
                        },
                        () -> {
                            IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
                            IntakeMotor.stopMotor();


                        }
                )
                        .until(() ->
                        {
                            //TODO: check for limit switch
                            return isPressed() || RobotContainer.Driver.getRightTriggerAxis() < 0.5;
                        }),
                Command_Rumble().alongWith(Command_SetPivotPosition(EPivotPosition.stowed))

        );
    }

    /*
    Command for controlling the rumble of the controller for 75 milliseconds
    */
    public Command Command_Rumble() {
        return startEnd(
                () -> {
                    rumbleTimer.start();
                    System.out.println("Rumblin");
                    RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
                },
                () -> {
                    RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                    rumbleTimer.stop();
                    rumbleTimer.reset();
                }
        )
                .until(() -> rumbleTimer.hasElapsed(0.75));
    }

    /*
    Command for intaking during auto, takes into account only the limit switch and not the controller
    (Might be a better way of doing this but I can't think of anything right now)
    */
    public Command Command_intakeauto() {
        return startEnd(
                () -> {
                    IntakePID.setReference(500, CANSparkBase.ControlType.kVelocity);
                },
                () -> {
                    IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
                    IntakeMotor.stopMotor();
                    Command_SetPivotPosition(EPivotPosition.stowed);
                }
        ).until(this::isPressed);
    }

    /*
        Commnad for stopping the motor, useful for if we need to stop the intake motor during auto
    */
    public Command Command_stopMotor() //use this in auto just incase we miss a note note
    {
        return runOnce(() ->
        {
            IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
            IntakeMotor.stopMotor();

        });
    }

    /*
    Command for a quick test of the intake for pid tuning
    */
    public Command Command_testInake(double speed) {

        return run(() ->
        {
            SmartDashboard.putNumber("Target Velocity", speed);
            if (speed == 600) {
                IntakePID.setReference(speed, CANSparkBase.ControlType.kVelocity);
            } else {
                IntakeMotor.stopMotor();
            }
        });
    }

    /*
    Command for outtaking the note into scoring location, will end when trigger is released
    */
    public Command Command_scoreSpeaker() {
        return
                startEnd(
                        () -> {
                            IntakePID.setReference(EOutakeType.speaker.RPM, CANSparkBase.ControlType.kVelocity);
                        },
                        () ->
                        {
                            IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
                            IntakeMotor.stopMotor();
                        }
                ).until(() -> RobotContainer.Driver.getLeftTriggerAxis() < 0.5);
    }

    public Command Command_scoreAmp()
    {
        return startEnd(
                () ->
                {
                    IntakePID.setReference(EOutakeType.amp.RPM, CANSparkBase.ControlType.kVelocity);
                },
                () ->
                {
                    IntakePID.setReference(0, CANSparkBase.ControlType.kVelocity);
                    IntakeMotor.stopMotor();
                }
        ).until(() -> RobotContainer.Driver.getHID().getLeftBumperReleased());
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

    public void periodic() {
        SmartDashboard.putNumber("pivot Position", PivotMotor.getPosition().getValue());
        SmartDashboard.putNumber("Actual Velocity", IntakeEncoder.getVelocity());
        SmartDashboard.putBoolean("is limit switched pressed", isPressed());
        UpdateConfigs();
    }

    private void PublishConfigs()
    {
        if (!TunePID) { return; }

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

        var dirty = false;

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

