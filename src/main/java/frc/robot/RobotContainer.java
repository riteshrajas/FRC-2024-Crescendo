// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import frc.robot.Commands.TurnInDirectionOfTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystems.LimelightVision;
import friarLib2.vision.LimelightCamera;
import frc.robot.subsystems.IntakeSusbsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // -- Controllers
    public static CommandXboxController Driver = new CommandXboxController(0);
    public static CommandXboxController Operator = new CommandXboxController(1);


    private double MaxSpeed = 2.11; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.RobotCentric AimRobot = new SwerveRequest.RobotCentric().withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // -- Camera's
    private LimelightCamera shooterCamera = new LimelightCamera();
    private PhotonCamera testCamera = new PhotonCamera("Test");

    // -- Subsystems 
    private final ArmSubsystem Arm = new ArmSubsystem();
    private final IntakeSusbsystem Intake = new IntakeSusbsystem();

    // -- Auto
    // private SwerveAutoBuilder AutoBuilder = null;
//    private final SendableChooser<Command> AutoChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;


    public RobotContainer()
    {
        autoChooser = AutoBuilder.buildAutoChooser();
        ConfigureBindings();
        SetDefaultCommands();
        // ConfigureSwerveAutoBuilder();
        // ConfigureAutoCommands();
        SmartDashboard.putData("Auto Chooser", autoChooser);


    }



    public Command GetAutonomousCommand()
    {
        return autoChooser.getSelected();
    }


    private void ConfigureBindings()
    {
        // ----------------------------------------------------------------------------------------
        // -- Driver
        // ----------------------------------------------------------------------------------------


        // -- Auto Turn


//        new Trigger(OI.DriverLeft::getTrigger).whileTrue(new TurnInDirectionOfTarget(_Drive));

        // Zero IMU
//       new Trigger(OI.DriverLeft::getTop).onTrue(new InstantCommand(IMU::zeroIMU));

        Driver.povUp().onTrue(Intake.Command_testInake(600));
        Driver.povUp().onFalse(Intake.Command_testInake(0));


        // ----------------------------------------------------------------------------------------
        // -- Operator
        // ----------------------------------------------------------------------------------------

        /********** Testing arm code **********/
        Operator.a().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed)); //stowed
        Operator.b().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_subwoofer)); //shot speaker
        Operator.x().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp)); //amp
        Operator.y().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.trap)); //trap



        /********** Intaking **********/
//        Operator.rightTrigger().whileTrue(
//                Commands.parallel(
//                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed),
//                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.intake))
//                .andThen(Intake.Command_IntakeNote())
//        );
//
//        Operator.rightTrigger().onFalse(Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed));

        /********** Outtake **********/
//        Operator.rightBumper().onTrue()


        /********** Scoring **********/
//       Operator.a().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed))); //stow
//       Operator.b().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_subwoofer), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.shoot_speaker))); //shoot speaker
//       Operator.x().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.amp))); // shoot amp
//       Operator.y().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.trap), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.trap))); // place trap

        /********** Climbing **********/
        Operator.leftStick().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_firstpos), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed)));
        Operator.rightStick().onTrue(Commands.parallel(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_secondpos), Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.stowed)));


        /********** Vision **********/
        // Operator.back().onTrue(LimelightVision.SetPipelineCommand(0).ignoringDisable(true));
        Operator.start().onTrue(LimelightVision.SetPipelineCommand(1).ignoringDisable(true));
//        Operator.rightStick().onTrue(LimelightVision.SetPipelineCommand(2).ignoringDisable(true));


    }


    private void SetDefaultCommands()
    {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-Driver.getLeftY() * MaxSpeed) // Drive forward with
                        // negative Y (forward)
                        .withVelocityY(-Driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-Driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ).ignoringDisable(true));

        Driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        Driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-Driver.getLeftY(), -Driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        Driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
        Driver.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(-0.5)));
        Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0.5)));
        Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(-0.5)));
        Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0.5)));
        Driver.povDown().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        Driver.povRight().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
        Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
        Driver.rightBumper().whileTrue(drivetrain.applyRequest(() -> AimRobot.withRotationalRate(LimelightVision.shooterCamera.getBestTarget().getX()))); // TODO: I don't know if this will work (if the robot has no target does it return 0?)
        Driver.leftTrigger().whileTrue(drivetrain.applyRequest(() -> {
            if (LimelightVision.shooterCamera.getBestTarget().getX() > 0.1) {
                return AimRobot.withRotationalRate(LimelightVision.shooterCamera.getBestTarget().getX() * -1);
            } else if (LimelightVision.getInchesFromTarget() > 5) {
                return forwardStraight.withVelocityX(0).withVelocityY(0.5);
            } else {
                return forwardStraight.withVelocityX(0).withVelocityY(0);
            }
        }));
        
        //Driver.rightBumper().whileTrue(drivetrain.applyRequest(() -> AimRobot.withRotationalRate(testCamera)));


//            Driver.leftBumper().whileTrue(drivetrain.applyRequest(() -> AimRobot.withRotationalRate(LimelightVision.shooterCamera.getBestTarget().getX()));



    }


    private void ConfigureAutos()
    {
        autoChooser.addOption("mid", drivetrain.getAutoPath("3 note middle") );
    }


}


