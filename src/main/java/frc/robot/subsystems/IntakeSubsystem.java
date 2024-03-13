package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;
import frc.robot.RobotContainer;
//*TODO: Find acutal poses, work out motor configurations, do commmands  and logic for moving pivot



public class IntakeSubsystem extends SubsystemBase
{

    public enum EPivotPosition
    {
        Stowed(PivotLimitReverse + PivotLimitReverseBuffer),
        Intake(-0.05),
        Shoot_speaker(PivotLimitReverse),
        Amp(0.075),
        Trap(PivotLimitReverse + PivotLimitReverseBuffer),
        Climb(-0.25);

        private final double Rotations;

        EPivotPosition(double rotations) {
            Rotations = rotations;
        }
    }

    public enum EOutakeType
    {
        amp(-0.5),
        speaker(-1), // known value = -60,  to testing value: 74

        trap(0);

        private final double DutyCycle;

        EOutakeType(double dutyCycle) {
            DutyCycle = dutyCycle;
        }
    }

    private final double PivotTolerance = 15.0 / 360.0;
    static private final double PivotLimitForward = 0.325;
    static private final double PivotLimitReverse = -0.31;

    // Since we zero on the hard stop, add this buffer to when going home so we don't slam into the stop.
    static private final double PivotLimitReverseBuffer = 0.02;
    private final double TEMP_IntakeDutyCycle = 0.4;


    // -- Motors
    private TalonFX PivotMotor;
    private TalonFX IntakeMotor;
    private CANSparkFlex FeederMotor;
    private SparkPIDController FeederMotorPID;


    // -- Pheonix Requests
    private final MotionMagicExpoTorqueCurrentFOC PivotRequest = new MotionMagicExpoTorqueCurrentFOC(0);

//    private final VelocityTorqueCurrentFOC IntakeRequest = new VelocityTorqueCurrentFOC(0);

    private final DutyCycleOut IntakeRequest = new DutyCycleOut(0);


    double lastCurrent = 0;
    int currentSpikeCount = 0;

    public IntakeSubsystem()
    {
        CreatePivotMotor();
        CreateIntakeMotor();
        CreateFeederMotor();

    }

    // --------------------------------------------------------------------------------------------
    // -- Pivot Motor
    // --------------------------------------------------------------------------------------------
    private void CreatePivotMotor()
    {
        PivotMotor = new TalonFX(Constants.CanivoreBusIDs.IntakePivot.GetID(), Constants.CanivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(1500)
                .withKI(0)
                .withKD(200)
                .withKS(10)
                .withKA(0)
                .withKV(0)
                .withKG(22));

        configs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(4)
                .withMotionMagicExpo_kA(5)
                .withMotionMagicExpo_kV(5)
                .withMotionMagicJerk(1000));

        configs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(60));


        configs.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(PivotLimitForward)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(PivotLimitReverse));

        PivotMotor.getConfigurator().apply(configs);
        PivotMotor.setPosition(PivotLimitReverse);
    }

    // --------------------------------------------------------------------------------------------
    // -- Intake Motor
    // --------------------------------------------------------------------------------------------
    private void CreateIntakeMotor()
    {
        IntakeMotor = new TalonFX(Constants.CanivoreBusIDs.IntakeMotor.GetID(), Constants.CanivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(10)
                .withKI(0)
                .withKD(0)
                .withKS(40)
                .withKA(0)
                .withKV(0.25)
                .withKG(0));

        IntakeMotor.getConfigurator().apply(configs);

        IntakeMotor.stopMotor();
    }

    // --------------------------------------------------------------------------------------------
    // -- Feeder Motor
    // --------------------------------------------------------------------------------------------
    private void CreateFeederMotor()
    {
        FeederMotor = new CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless);

        FeederMotor.restoreFactoryDefaults();
        FeederMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        FeederMotor.setInverted(true);

        FeederMotorPID = FeederMotor.getPIDController();

        FeederMotorPID.setP(0.05);
        FeederMotorPID.setI(0.0000001);
        FeederMotorPID.setD(0.01357);
        FeederMotorPID.setIZone(0);
        FeederMotorPID.setFF(0.000015);
        FeederMotorPID.setOutputRange(-1, 1);

        FeederMotor.burnFlash();

        FeederMotor.stopMotor();
    }


    public double GetPivotPos() {
        return PivotMotor.getPosition().getValue();
    }

    public Command Command_SetPivotPosition(EPivotPosition position)
    {
        return run(() ->
        {
            SmartDashboard.putNumber("Intake.PivotTarget", position.Rotations);
            PivotMotor.setControl(PivotRequest.withPosition(position.Rotations));
        })
        .until(() ->
        {
            double actualRotation = PivotMotor.getPosition().getValue();
            return MathUtil.isNear(position.Rotations, actualRotation, PivotTolerance);
        });
    }

    public Command Command_SetCoastMode(NeutralModeValue mode)
    {
        return runOnce(() -> PivotMotor.setNeutralMode(mode)).ignoringDisable(true);
    }

    public Command Command_PreIntakeSpinUp() {
        // TODO: This should be a startEnd command that stops the motor when it finishes in case the command gets interrupted,
        //       but we can't do that right now due to how the intake and arm subsystems are divided up. Until then we'll just
        //       spin up the wheels and rely on the invoking command sequence to stop the intake.
        return runOnce(() ->
        {
            IntakeMotor.setControl(IntakeRequest.withOutput(TEMP_IntakeDutyCycle));
            //IntakeMotor.setControl(IntakeRequest.withVelocity(35));
        });
    }

    public Command Command_IntakeNote()
    {
        return startEnd(
            () -> {
                currentSpikeCount = 0;
                lastCurrent = IntakeMotor.getStatorCurrent().getValue();
                IntakeMotor.setControl(IntakeRequest.withOutput(TEMP_IntakeDutyCycle));
            },
            () -> {
                IntakeMotor.stopMotor();
                PivotMotor.setControl(PivotRequest.withPosition(EPivotPosition.Stowed.Rotations));
            }
        )
        .until(() ->
        {
            double curCurrent = IntakeMotor.getStatorCurrent().getValue();
            SmartDashboard.putNumber("Intake.CurrentDelta", curCurrent - lastCurrent);

            if (curCurrent - lastCurrent > 25)
            {
                currentSpikeCount++;
            }

            lastCurrent = curCurrent;

            if (currentSpikeCount >= 1)
            {
                CommandScheduler.getInstance().schedule(
                    Commands.startEnd(
                        () -> RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1),
                        () -> RobotContainer.Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)
                    ).withTimeout(0.5));
                return true;
            }
            return false;
        })
        .andThen(Command_FeederTakeNote());
    }

    public Command Command_FeederTakeNote()
    {
        var feederMotorIntakeSpeed = -0.075;

       return Commands.sequence(
           run(() -> {
               FeederMotorPID.setReference(feederMotorIntakeSpeed, CANSparkBase.ControlType.kDutyCycle);
               IntakeMotor.setControl(IntakeRequest.withOutput(3));
           }).withTimeout(0.2),
           startEnd(
               () -> {
                   currentSpikeCount = 0;
                   lastCurrent = FeederMotor.getOutputCurrent();
                   FeederMotorPID.setReference(feederMotorIntakeSpeed, CANSparkBase.ControlType.kDutyCycle);
               },
               () -> {
                   FeederMotor.stopMotor();
                   IntakeMotor.stopMotor();
               }
           )
           .until(() -> {
               double curCurrent = FeederMotor.getOutputCurrent();
               SmartDashboard.putNumber("Intake.CurrentDelta", curCurrent - lastCurrent);
               if (curCurrent - lastCurrent > 5) {
                   currentSpikeCount++;
               }
               return currentSpikeCount >= 1;
           }),
           startEnd(
               () -> FeederMotorPID.setReference(feederMotorIntakeSpeed, CANSparkBase.ControlType.kDutyCycle),
               () -> FeederMotor.stopMotor())
               .withTimeout(0.15)
        );
    }

    public Command Command_MoveNote(boolean forward)
    {
        var dutyCycle = forward ? -0.15: 0.15;

        return startEnd(
            () -> {
                //FeederMotorPID.setReference(velocity * 60, CANSparkBase.ControlType.kVelocity);
                IntakeMotor.setControl(IntakeRequest.withOutput(dutyCycle));

            },
            () -> {
                IntakeMotor.stopMotor();
                FeederMotor.stopMotor();
            }
        );
    }

    public Command Command_Outtake(EOutakeType outtakeType)
    {
        return Commands.sequence(
           run(() -> IntakeMotor.setControl(IntakeRequest.withOutput(outtakeType.DutyCycle)))
               .withTimeout(0.25),
           run(() -> FeederMotor.set(0.5))
               .withTimeout(0.25)
        );
    }


    public Command Command_StopIntake() //use this in auto just in case we miss a note
    {
        return runOnce(() ->
        {
            IntakeMotor.stopMotor();
            FeederMotor.stopMotor();
        });
    }


    public Command Command_ZeroPivotEncoder()
    {
        return runOnce(() -> PivotMotor.setPosition(PivotLimitReverse))
            .ignoringDisable(true);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Intake.CurrentSpikeCount", currentSpikeCount);
        SmartDashboard.putNumber("Intake.PivotPosition", PivotMotor.getPosition().getValue());
        SmartDashboard.putNumber("Intake.IntakeCurrent", IntakeMotor.getStatorCurrent().getValue());
        SmartDashboard.putNumber("Intake.FeederCurrent", FeederMotor.getOutputCurrent());

    }



}

