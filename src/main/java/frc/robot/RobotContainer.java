// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSusbsystem;
import frc.robot.subsystems.SwerveSubsystem;
import friarLib2.math.LookupTable;
import friarLib2.vision.LimelightCamera;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    static public final PowerDistribution PDH = new PowerDistribution();

    // --------------------------------------------------------------------------------------------
    // -- Controllers
    // --------------------------------------------------------------------------------------------
    public static CommandXboxController Driver = new CommandXboxController(0);
    public static CommandXboxController Operator = new CommandXboxController(1);



    // --------------------------------------------------------------------------------------------
    // -- Tuning Values
    // --------------------------------------------------------------------------------------------
    private final double MaxRotationsPerSecond = 0.75;
    private LookupTable ThrottleLut = new LookupTable.Normalized()
            .AddValue(0.1, 0.0) // dead-band
            .AddValue(0.35, 0.05)
            .AddValue(0.75, 0.2);

    private LookupTable TurnLut = new LookupTable.Normalized()
            .AddValue(0.1, 0.0) // dead-band
            .AddValue(0.35, 0.05)
            .AddValue(0.75, 0.2);
    
    // --------------------------------------------------------------------------------------------
    // -- Subsystems~
    // --------------------------------------------------------------------------------------------
    public final SwerveSubsystem drivetrain = TunerConstants.DriveTrain;
    private final ArmSubsystem Arm = new ArmSubsystem();
    private final IntakeSusbsystem Intake = new IntakeSusbsystem();
    private final PoseManager Pose = new PoseManager(Arm, Intake);
    
    // --------------------------------------------------------------------------------------------
    // -- Cameras
    // --------------------------------------------------------------------------------------------
    private LimelightCamera shooterCamera = new LimelightCamera();

    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Drive requests
    // --------------------------------------------------------------------------------------------
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveTowardsAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.RobotCentric AimRobot = new SwerveRequest.RobotCentric().withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    // --------------------------------------------------------------------------------------------
    // -- Shared Commands
    // --------------------------------------------------------------------------------------------

    private Command Command_IntakeNoteSequence;

    // --------------------------------------------------------------------------------------------
    // -- Auto Chooser
    // --------------------------------------------------------------------------------------------
    private SendableChooser<Command> autoChooser;

    // --------------------------------------------------------------------------------------------
    // -- State
    // --------------------------------------------------------------------------------------------
    private boolean RotationModeIsRobotCentric = true;


    public RobotContainer()
    {
        CreateSharedCommands();

        NamedCommands.registerCommand("Arm Score", Pose.Command_GoToPose(PoseManager.EPose.Speaker));
        NamedCommands.registerCommand("Arm Score_AMP", Pose.Command_GoToPose(PoseManager.EPose.Amp));

        NamedCommands.registerCommand("Arm Stow", Pose.Command_GoToPose(PoseManager.EPose.Stowed));

        NamedCommands.registerCommand("Intake Note", Command_IntakeNoteSequence);
        NamedCommands.registerCommand("Stop Intake motors", Intake.Command_StopIntake());
        NamedCommands.registerCommand("Shoot Speaker", Intake.Command_OuttakeAuto(IntakeSusbsystem.EOutakeType.speaker));
        NamedCommands.registerCommand("Shoot Amp", Intake.Command_OuttakeAuto(IntakeSusbsystem.EOutakeType.amp));

        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.addOption("mid", drivetrain.getAutoPath("AMP Auto") );

        SmartDashboard.putData("Auto Chooser", autoChooser);

        SetDefaultCommands();
        
        ConfigureDriverBindings();
        ConfigureOperatorBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
   }

    private void CreateSharedCommands()
    {
        Command_IntakeNoteSequence =
            Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed)
            .andThen(Intake.Command_PreIntakeSpinUp())
            .andThen(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Intake))
            .andThen(Intake.Command_IntakeNote())
            .andThen(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Stowed));
    }


    private void ConfigureAutoCommands()
    {

    }
    
    public Command GetAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    
    
    private void SetDefaultCommands()
    {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(this::GetDefaultDriveRequest)
                .ignoringDisable(true));
    }

    


    // --------------------------------------------------------------------------------------------
    // -- Driver
    // --------------------------------------------------------------------------------------------
    private void ConfigureDriverBindings()
    {

        // Slowdrive relative to bot pose
        Driver.povUp().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5))); // Fine-tune control forwards
        Driver.povDown().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5))); // Fine-tune control backwards
        Driver.povRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(-.5))); // Fine-tune control right
        Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(.5))); // Fine-tune control left

        Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(-.5))); // Fine-tune control diagonally up and right
        Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(.5))); // Fine-tune control diagonally up and left
        Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(.5))); // Fine-tune control diagonally down and left
        Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(-.5))); // Fine-tune control diagonally down and right

        Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        Driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        Driver.rightTrigger()
            .whileTrue(Command_IntakeNoteSequence)
            .onFalse(Intake.Command_StopIntake() // Stop intake here is temporary until we can refactor the subsystems to make the intake motor separate from the pivot motor
            .andThen(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Stowed)));



        Driver.leftTrigger().whileTrue(
                // TODO: automatially decide the outtake type based on vision
                Intake.Command_Outtake(IntakeSusbsystem.EOutakeType.amp)
        );

//        Driver.x().onTrue(Commands.run(() -> RotationModeIsRobotCentric = !RotationModeIsRobotCentric));
    }
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Operator
    // --------------------------------------------------------------------------------------------
    private void ConfigureOperatorBindings()
    {
        Operator.start().onTrue(Arm.Command_ZeroArmEncoder().alongWith(Intake.Command_ZeroPivotEncoder().ignoringDisable(true)));


        //Operator.a().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed));
        //Operator.b().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_speaker));
        //Operator.x().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp));
        //Operator.y().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.trap));

        //Operator.povDown().onTrue(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Intake));
        //Operator.povRight().onTrue(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Shoot_speaker));
        //Operator.povLeft().onTrue(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.amp));
        //Operator.povUp().onTrue(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Stowed));

        // -- Testing arm code


        /**
         * Zach-
         *
         *
         * Testing using Limelight or Operator to set the robot pose/ arm & intake
         * to certain spot
         */

        Operator.back().whileTrue(Arm.Command_ManualArmControl());

        Operator.a().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Stowed));
        Operator.b().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Intake));
        Operator.x().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Amp));
        Operator.y().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Speaker));


        Operator.rightBumper().onTrue(Pose.Command_AutoPose());



        // -- Intaking
//        Operator.rightTrigger().whileTrue(
//                Commands.parallel(
//                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed),
//                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.intake))
//                .andThen(Intake.Command_IntakeNote())
//        );
        

        // Operator.rightTrigger().onFalse(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed));

        // -- Outtake
        // Operator.rightBumper().onTrue()

        // -- Scoring
        // Operator.a().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed))); //stow
        // Operator.b().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_subwoofer), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.shoot_speaker))); //shoot speaker
        // Operator.x().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.amp))); // shoot amp
        // Operator.y().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.trap), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.trap))); // place trap

        // -- Climbing
        // Operator.leftStick().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_firstpos), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed)));
        // Operator.rightStick().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_secondpos), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed)));


        // -- Vision
        // Operator.back().onTrue(LimelightVision.SetPipelineCommand(0).ignoringDisable(true));
        //Operator.start().onTrue(LimelightVision.SetPipelineCommand(1).ignoringDisable(true));
        // Operator.rightStick().onTrue(LimelightVision.SetPipelineCommand(2).ignoringDisable(true));


    }

    private SwerveRequest GetDefaultDriveRequest()
    {
        double y = Driver.getLeftX();
        double x = Driver.getLeftY();

        double deflection = Math.sqrt(x*x + y*y);
        double deflectionLut = ThrottleLut.GetValue(deflection);
        if (deflectionLut == 0)
        {
            return driveFieldCentric
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(DefaultDriveRotationRate());
        }

        double finalX = x * deflectionLut * TunerConstants.kSpeedAt12VoltsMps;
        double finalY = y * deflectionLut * TunerConstants.kSpeedAt12VoltsMps;

        SmartDashboard.putNumber("XRaw", x);
        SmartDashboard.putNumber("YRaw", y);

        SmartDashboard.putNumber("Deflection", deflection);
        SmartDashboard.putNumber("Deflectionlut", deflectionLut);

        SmartDashboard.putNumber("Xfinal", finalX);
        SmartDashboard.putNumber("Yfinal", finalY);

        if (RotationModeIsRobotCentric)
        {
            return driveFieldCentric
                    .withVelocityX(-finalX)
                    .withVelocityY(-finalY)
                    .withRotationalRate(DefaultDriveRotationRate());
        }
        else
        {
            return driveRobotCentric
                    .withVelocityX(-finalX)
                    .withVelocityY(-finalY)
                    .withRotationalRate(DefaultDriveRotationRate());
        }
    }

    private double DefaultDriveRotationRate()
    {
        if (Driver.getHID().getLeftBumper())
        {
            var target = Vision.GetBestTarget();
            if (target != null)
            {
                return target.tx * -0.1;
            }
        }

        double magicScalar = 0.02; // The steering gains seem borked, but I ran out of time to figure out how to tune them correctly, so here's a magic number to get it to turn decently.

        double finalAngVelocity = -TurnLut.GetValue(Driver.getRightX()) * MaxRotationsPerSecond * 360;
        SmartDashboard.putNumber("FinalAngVel", Math.toRadians(finalAngVelocity));
        return finalAngVelocity * magicScalar;
    }
}


