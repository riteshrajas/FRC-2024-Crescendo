package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Makes the robot follow the pathweaver JSON passed in through the constructor
 */
public class FollowTrajectory extends CommandBase {

    private final DriveSubsystem drive;

    private final boolean resetOdometry;

    private final HolonomicDriveController holonomicController;
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;

    private final Field2d field = new Field2d();

    /**
     * @param drive The drive subsystem
     */
    public FollowTrajectory (
        DriveSubsystem drive,
        String trajectoryJSON,
        double maxSpeed,
        double maxAccel,
        boolean resetOdometry
    ) {
        this.drive = drive;
        addRequirements(drive);

        this.resetOdometry = resetOdometry;

        // The holonomic controller uses the current robot pose and the target pose (from the trajectory) to calculate the required ChassisSpeeds to get to that location
        // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html for more details
        holonomicController = new HolonomicDriveController(
            Constants.Drive.HOLONOMIC_CONTROLLER_PID_X, 
            Constants.Drive.HOLONOMIC_CONTROLLER_PID_Y, 
            Constants.Drive.HOLONOMIC_CONTROLLER_PID_THETA
        );
        // Set the range where the holonomic controller considers itself at its target location
        holonomicController.setTolerance(new Pose2d(new Translation2d(.09, .09), Rotation2d.fromDegrees(3)));

        trajectory = openTrajectoryFromJSON(trajectoryJSON, maxSpeed, maxAccel); //Load the Pathplanner trajectory
        PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()); // Mirror if we're on the red alliance

        SmartDashboard.putData("Holonomic target", field);
    }

    public FollowTrajectory (DriveSubsystem drive, String trajectoryJSON, boolean resetOdometry) {
        this(drive, trajectoryJSON, Constants.Drive.MAX_AUTON_SPEED, Constants.Drive.MAX_AUTON_ACCELERATION, resetOdometry);
    }

    public FollowTrajectory (DriveSubsystem drive, String trajectoryJSON) {
        this(drive, trajectoryJSON, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PathPlannerState initialState = (PathPlannerState) trajectory.sample(0);

        if (resetOdometry) {
            drive.HardResetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation)); // Re-zero the robot's odometry
        }

        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PathPlannerState goal = (PathPlannerState) trajectory.sample(timer.get()); //Find the target pose for the current time

        //Use the holonomic drive controller to calculate the required chassis speeds to follow the trajectory
        drive.setChassisSpeeds(holonomicController.calculate(
            drive.GetPose(),
            goal, 
            goal.holonomicRotation
        ));
        
        SmartDashboard.putString("Holonomic controller error", drive.GetPose().minus(goal.poseMeters).toString());
        SmartDashboard.putNumber("Holonomic x error",  drive.GetPose().minus(goal.poseMeters).getTranslation().getX());
        SmartDashboard.putNumber("Holonomic y error",  drive.GetPose().minus(goal.poseMeters).getTranslation().getY());
        SmartDashboard.putNumber("Holonomic theta target", goal.holonomicRotation.getDegrees());
        SmartDashboard.putNumber("Holonomic theta error", Constants.Drive.HOLONOMIC_CONTROLLER_PID_THETA.getPositionError());

        field.setRobotPose(new Pose2d(goal.poseMeters.getTranslation(), goal.holonomicRotation));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stopChassis();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return holonomicController.atReference() && timer.get() >= trajectory.getTotalTimeSeconds();
    }

    /**
     * Read the JSON output from pathweaver and convert it to a trajectory
     * object using the specified speed limits
     *
     * @param JSONName the name of the JSON stored in the deploy/output directory, e.x. "bounceLeg1.wpilib.json"
     */
    private PathPlannerTrajectory openTrajectoryFromJSON (String JSONName, double maxSpeed, double maxAccel) {
        return PathPlanner.loadPath(JSONName, maxSpeed, maxAccel);
    }

    /**
     * Read the JSON output from pathweaver and convert it to a trajectory
     * object using the default speed limits
     *
     * @param JSONName the name of the JSON stored in the deploy/output directory, e.x. "bounceLeg1.wpilib.json"
     */
    private PathPlannerTrajectory openTrajectoryFromJSON (String JSONName) {
        return openTrajectoryFromJSON(JSONName, Constants.Drive.MAX_AUTON_SPEED, Constants.Drive.MAX_AUTON_ACCELERATION);
    }
}