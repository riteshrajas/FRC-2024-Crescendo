// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import friarLib2.math.LookupTable;



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

    private DriverStation.Alliance DefaultAlliance = DriverStation.Alliance.Blue;

    private DriverStation.Alliance CurrentAlliance = DriverStation.getAlliance()
                                                                  .get();

    // --------------------------------------------------------------------------------------------
    // -- Subsystems~
    // --------------------------------------------------------------------------------------------
    public final SwerveSubsystem drivetrain = TunerConstants.DriveTrain;
    public final ArmSubsystem Arm = new ArmSubsystem();
    public final IntakeSubsystem Intake = new IntakeSubsystem();
    public final PoseManager Pose = new PoseManager(Arm, Intake);
    
    
    
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
    // -- Auto Chooser
    // --------------------------------------------------------------------------------------------
    private final SendableChooser<Command> autoChooser;

    // --------------------------------------------------------------------------------------------
    // -- State
    // --------------------------------------------------------------------------------------------
    private boolean RotationModeIsRobotCentric = false;

    public RobotContainer()
    {
        ConfigureAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("SimpleForward", Commands.run(() -> driveFieldCentric.withVelocityX(1)).withTimeout(2));

        SetDefaultCommands();
        
        ConfigureDriverBindings();
        ConfigureOperatorBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
   }

//   Command Command_AutoLineupSequence()
//   {
//
//       var target = Vision.GetBestTarget();
//
//       double tX = 0.0;
//       double tY = 0.0;
//       Rotation2d yaw = new Rotation2d(tX, tY);
//
//       float distance = 0; // wanted dist
//
//       if (target != null)
//       {
//           if (distance != Vision.LastDist)
//           {
//               while (target.tx != tX)
//               {
//                   return drivetrain.applyRequest(() -> driveFieldCentric.withRotationalRate(-target.tx));
//               }
//           }
//
//       }
//
//       return drivetrain.applyRequest(() -> driveFieldCentric);
//
//   }

   Command Command_TestLineup()
   {
       return Commands.runEnd(
           () ->
           {
               double deltaX;
               double deltaY;
               double deltaT;

               var target = Vision.GetBestTarget();
               if (target == null) { return;}

               var pose = LimelightHelpers.getBotPose2d("");

               if (CurrentAlliance == DefaultAlliance)
               {
                   deltaX = pose.getX() - -6.45;
               } else {
                   deltaX = pose.getX() - 6.45;
               }
               deltaY = pose.getY() - 0;
               deltaT = pose.getRotation()
                            .getDegrees() - 90;

               SmartDashboard.putNumber("AutoLineup.DeltaX", deltaX);
               SmartDashboard.putNumber("AutoLineup.DeltaY", deltaY);
               SmartDashboard.putNumber("AutoLineup.DeltaT", deltaT);

               drivetrain.applyRequest(() -> driveFieldCentric
                   .withVelocityX(deltaX)
                   //.withVelocityY(deltaY)
                   .withRotationalRate(deltaT));
           },
           () -> drivetrain.applyRequest(() -> brake), drivetrain
       );

   }

   Command Command_IntakeNoteSequence(boolean fromSource)
    {
        return Commands.sequence(
            Arm.Command_SetPosition(fromSource ? ArmSubsystem.EArmPosition.Source : ArmSubsystem.EArmPosition.Stowed),
            Intake.Command_IntakeNote(fromSource)
        );
    }

    public Command Command_AutoPose()
    {
        return Commands.runOnce(() -> Pose.Command_GoToPose(Pose.GetPoseForCurrentTag()));
    }

    private void ConfigureAutoCommands()
    {
        NamedCommands.registerCommand("Arm Score", Pose.Command_GoToPose(PoseManager.EPose.Speaker));
        NamedCommands.registerCommand("Arm Stow", Pose.Command_GoToPose(PoseManager.EPose.Stowed));
        NamedCommands.registerCommand("Intake Note", Command_IntakeNoteSequence(false));
        NamedCommands.registerCommand("Stop Intake motors", Intake.Command_StopIntake());
        NamedCommands.registerCommand("Shoot Speaker", Intake.Command_Outtake(IntakeSubsystem.EOutakeType.speaker).withTimeout(0.5).andThen(Intake.Command_StopIntake()));
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

        //Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //Driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

        Driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative).ignoringDisable(true));
        Driver.a().whileTrue(Command_TestLineup());

        Driver.rightTrigger().onTrue(Command_IntakeNoteSequence(false));
        Driver.rightTrigger().onFalse(Commands.runOnce(() -> Intake.RequestCancelIntake()));

        Driver.leftTrigger().onTrue(
            Commands.sequence(
                //Pose.Command_GoToPose(PoseManager.EPose.Speaker),
                Intake.Command_Outtake(IntakeSubsystem.EOutakeType.speaker),
                Pose.Command_GoToPose(PoseManager.EPose.Stowed))
        );

        Driver.leftBumper().onTrue(
            Commands.sequence(
                Intake.Command_Outtake(IntakeSubsystem.EOutakeType.amp),
                Pose.Command_GoToPose(PoseManager.EPose.Stowed))
        );
    }
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Operator
    // --------------------------------------------------------------------------------------------
    private void ConfigureOperatorBindings()
    {
        Operator.start().onTrue(Arm.Command_ZeroArmEncoder().alongWith(Intake.Command_ZeroPivotEncoder().ignoringDisable(true)));
        Operator.back().whileTrue(Arm.Command_ManualArmControl());



        Operator.a().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Stowed));
        Operator.b().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Intake));
        Operator.x().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Amp));
        Operator.y().onTrue(Pose.Command_GoToPose(PoseManager.EPose.Speaker));

        Operator.povUp().onTrue(Pose.Command_GoToPose(PoseManager.EPose.PreClimb));
        Operator.povDown().onTrue(Arm.Command_Climb());

        Operator.povLeft().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Brake).alongWith(Intake.Command_SetNeutralMode(NeutralModeValue.Brake)));
        Operator.povRight().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Coast).alongWith(Intake.Command_SetNeutralMode(NeutralModeValue.Coast)));

        Operator.rightBumper().whileTrue(Intake.Command_MoveNote(false));
        Operator.leftBumper().whileTrue(Intake.Command_MoveNote(true));

        Operator.leftTrigger().onTrue(Intake.Command_FeederTakeNote(false));

        Operator.rightTrigger().onTrue(Command_IntakeNoteSequence(true));
        Operator.rightTrigger().onFalse(Pose.Command_GoToPose(PoseManager.EPose.Stowed).andThen(Commands.runOnce(() -> Intake.RequestCancelIntake())));
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
            return driveRobotCentric
                .withVelocityX(-finalX)
                .withVelocityY(-finalY)
                .withRotationalRate(DefaultDriveRotationRate());
        }
        else
        {
            return driveFieldCentric
                .withVelocityX(-finalX)
                .withVelocityY(-finalY)
                .withRotationalRate(DefaultDriveRotationRate());
        }
    }

    private double DefaultDriveRotationRate()
    {
        if (Driver.getHID().getRightBumper()) //Dumb solution for note tracking but it works
        {
            if (LimelightHelpers.getCurrentPipelineIndex("") == 1)
            {
                var target = Vision.GetBestNoteTarget();
                if (target != null)
                {
                    return target.tx * -0.1;
                }
            }
            else {
                var target = Vision.GetBestTarget();
                if (target != null)
                {
                    return target.tx * -0.1;
                }
            }
        }

        double magicScalar = 0.02; // The steering gains seem borked, but I ran out of time to figure out how to tune them correctly, so here's a magic number to get it to turn decently.

        double finalAngVelocity = -TurnLut.GetValue(Driver.getRightX()) * MaxRotationsPerSecond * 360;
        SmartDashboard.putNumber("FinalAngVel", Math.toRadians(finalAngVelocity));
        return finalAngVelocity * magicScalar;
    }


}
