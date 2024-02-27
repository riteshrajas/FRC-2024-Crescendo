package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSusbsystem;

public class PoseManager
{
    enum EPose
    {
        None,
        Stowed,
        Intake,
        Amp,
        Speaker,
        PreClimb
    }

    private ArmSubsystem Arm;
    private IntakeSusbsystem Intake;

    public PoseManager(ArmSubsystem arm, IntakeSusbsystem intake)
    {
        Arm = arm;
        Intake = intake;
    }

    private EPose GetPoseForCurrentTag()
    {
        var target = Vision.GetBestTarget();
        if (target == null) { return EPose.None; }

        if (target.fiducialID == 4 || target.fiducialID == 7)
        {
            return EPose.Speaker;
        }
        else if (target.fiducialID == 5 || target.fiducialID == 6)
        {
            return EPose.Amp;
        }
        else if (target.fiducialID >= 11 || target.fiducialID <= 16)
        {
            return EPose.PreClimb;
        }

        return EPose.None;
    }

    public Command Command_GoToPose(EPose pose)
    {
        if (pose == EPose.None) { return Commands.none(); }

        if (pose == EPose.Stowed)
        {
            return Commands.parallel(
                Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed),
                Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Stowed)
            );
        }

        if (pose == EPose.Intake)
        {
            return Commands.parallel(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.stowed),
                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Intake)
            );
        }

        if (pose == EPose.Amp)
        {
            return Commands.parallel(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.amp),
                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.amp)
            );
        }

        if (pose == EPose.Speaker)
        {
            return Commands.parallel(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.shoot_speaker),
                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Shoot_speaker)
            );
        }

        if (pose == EPose.PreClimb)
        {
            return Commands.parallel(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.climb_firstpos),
                    Intake.Command_SetPivotPosition(IntakeSusbsystem.EPivotPosition.Stowed)
            );
        }

        return Commands.none();
    }

    public Command Command_AutoPose()
    {
        return Command_GoToPose(GetPoseForCurrentTag());
    }

}