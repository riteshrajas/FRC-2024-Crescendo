// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import frc.robot.Commands.TurnInDirectionOfTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.HashMap;


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
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // -- Subsystems 
    private final ArmSubsystem Arm = new ArmSubsystem();

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



        // ----------------------------------------------------------------------------------------
        // -- Operator
        // ----------------------------------------------------------------------------------------
        // -- Arm
//        Operator.a().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed));
//        Operator.b().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_podium));
//        Operator.y().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_wing));
//        Operator.x().onTrue(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_firstpos));
        Operator.a().onTrue(new InstantCommand(() -> Arm.setTargetPosition(ArmSubsystem.EArmPosition.stowed)));
        Operator.b().onTrue(new InstantCommand(() -> Arm.setTargetPosition(ArmSubsystem.EArmPosition.shoot_podium)));
        Operator.y().onTrue(new InstantCommand(() -> Arm.setTargetPosition(ArmSubsystem.EArmPosition.shoot_wing)));
        Operator.x().onTrue(new InstantCommand(() -> Arm.setTargetPosition(ArmSubsystem.EArmPosition.climb_firstpos)));


        // -- Vision
        // Operator.back().onTrue(LimelightVision.SetPipelineCommand(0).ignoringDisable(true));
        Operator.start().onTrue(LimelightVision.SetPipelineCommand(1).ignoringDisable(true));
        Operator.rightStick().onTrue(LimelightVision.SetPipelineCommand(2).ignoringDisable(true));

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
        Driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

        Driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        Arm.setDefaultCommand(new RunCommand(Arm::runAutomatic, Arm));

    }

    private void ConfigureAutos()
    {
        autoChooser.addOption("mid", drivetrain.getAutoPath("3 note middle") );
    }


}


