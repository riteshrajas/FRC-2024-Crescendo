// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import org.photonvision.PhotonCamera;

import java.awt.*;
import java.awt.geom.Point2D;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // --------------------------------------------------------------------------------------------
    // -- Controllers
    // --------------------------------------------------------------------------------------------
    public static CommandXboxController Driver = new CommandXboxController(0);
    public static CommandXboxController Operator = new CommandXboxController(1);



    // --------------------------------------------------------------------------------------------
    // -- Tuning Values
    // --------------------------------------------------------------------------------------------
    private double MaxAngularRate = 2.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private LookupTable ThrottleLookup = new LookupTable.Normalized()
            .AddValue(0.07, 0.0) // deadband
            .AddValue(0.55, 0.05)
            .AddValue(0.8, 0.15);

    
    // --------------------------------------------------------------------------------------------
    // -- Subsystems~
    // --------------------------------------------------------------------------------------------
    public final SwerveSubsystem drivetrain = TunerConstants.DriveTrain;
    private final ArmSubsystem Arm = new ArmSubsystem();
    private final IntakeSusbsystem Intake = new IntakeSusbsystem();


    
    // --------------------------------------------------------------------------------------------
    // -- Cameras
    // --------------------------------------------------------------------------------------------
    private LimelightCamera shooterCamera = new LimelightCamera();
    private PhotonCamera testCamera = new PhotonCamera("Test");
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Drive requests
    // --------------------------------------------------------------------------------------------
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveTowardsAngle = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
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
    private boolean RotationModeIsRobotCentric = true;


    public RobotContainer()
    {
        autoChooser = AutoBuilder.buildAutoChooser();
        ConfigureAutoCommands();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        SetDefaultCommands();
        
        ConfigureDriverBindings();
        ConfigureOperatorBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    private void ConfigureAutoCommands()
    {
        autoChooser.addOption("mid", drivetrain.getAutoPath("3 note middle") );


        NamedCommands.registerCommand("Arm Score", Commands.parallel(
                  Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_subwoofer)
                , Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.shoot_speaker)));
        
        NamedCommands.registerCommand("Arm Stow", Commands.parallel(
                  Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed)
                , Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed)));
        
        NamedCommands.registerCommand("Intake Note", Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed).andThen(Intake.Command_intakeauto()));
        NamedCommands.registerCommand("stop Intake motors", Intake.Command_stopMotor());
        NamedCommands.registerCommand("Shoot Speaker", Intake.Command_scoreSpeaker());
    }
    
    public Command GetAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    
    
    private void SetDefaultCommands()
    {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        GetDefaultDriveRequest()
                        //.withVelocityX(-ThrottleLookup.GetValue(Driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                        //.withVelocityY(-ThrottleLookup.GetValue(Driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(DefaultDriveRotationRate()) // Drive counterclockwise with negative X (left)
                ).ignoringDisable(true));
    }

    


    // --------------------------------------------------------------------------------------------
    // -- Driver
    // --------------------------------------------------------------------------------------------
    private void ConfigureDriverBindings()
    {

        // Slowdrive relative to bot pose
        Driver.povUp().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(.5))); // Fine-tune control forwards
        Driver.povDown().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-.5))); // Fine-tune control backwards
        Driver.povRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(-.5))); // Fine-tune control right
        Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityY(.5))); // Fine-tune control left

        Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(.5).withVelocityY(-.5))); // Fine-tune control diagonally up and right
        Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(.5).withVelocityY(.5))); // Fine-tune control diagonally up and left
        Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-.5).withVelocityY(.5))); // Fine-tune control diagonally down and left
        Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-.5).withVelocityY(-.5))); // Fine-tune control diagonally down and right

        Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        Driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

//        Driver.rightTrigger().whileTrue(
//                Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed)
//                .andThen(Intake.Command_IntakeNote()));

        Driver.x().onTrue(Commands.run(() -> RotationModeIsRobotCentric = !RotationModeIsRobotCentric));
    }
    
    
    
    // --------------------------------------------------------------------------------------------
    // -- Operator
    // --------------------------------------------------------------------------------------------
    private void ConfigureOperatorBindings()
    {

        // -- Testing arm code
        Operator.a().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed));
        Operator.b().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_subwoofer));
        Operator.x().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp));
        Operator.y().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.trap));
        Operator.back().whileTrue(Arm.ManualArmControl());


        
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

    private SwerveRequest.FieldCentric GetDefaultDriveRequest()
    {

        double x = Driver.getLeftX();
        double y = Driver.getLeftY();

        double deflection = Math.sqrt(x * x + y * y);

        double theta = Math.abs(Math.toDegrees(Math.asin(y / deflection)));

        double xPercent = Math.abs(90 - theta) / 90;
        double yPercent = Math.abs(theta) / 90;

        double deflectionLut = ThrottleLookup.GetValue(deflection);

        double finalX = xPercent * deflectionLut * TunerConstants.kSpeedAt12VoltsMps * (x < 0 ? -1 : 1);
        double finalY = yPercent * deflectionLut * TunerConstants.kSpeedAt12VoltsMps * (y < 0 ? -1 : 1);

        SmartDashboard.putNumber("XRaw", x);
        SmartDashboard.putNumber("YRaw", y);

        SmartDashboard.putNumber("XPercent", xPercent);
        SmartDashboard.putNumber("YPercent", yPercent);

        SmartDashboard.putNumber("Deflection", deflection);
        SmartDashboard.putNumber("Deflectionlut", deflectionLut);

        SmartDashboard.putNumber("Theta", theta);

        SmartDashboard.putNumber("Xfinal", finalX);
        SmartDashboard.putNumber("Yfinal", finalY);

        //if (RotationModeIsRobotCentric)
        {
            return drive.withVelocityX(-finalX).withVelocityY(finalY);
        }
        //else
        //{
        //    return driveTowardsAngle.withVelocityX(-finalX).withVelocityY(finalY);
        //}
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
        return -ThrottleLookup.GetValue(Driver.getRightX()) * MaxAngularRate;
    }
}


