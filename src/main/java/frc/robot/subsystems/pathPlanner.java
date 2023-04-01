package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * 1
 * i what to move the robot frod 18 ft. 8inZ xcvfd SFZGHJJHDFS to grab a cub.
 * thin trn baKWRD to scor the cub.
 * 
 * 2
 * if posibl move on the chring bord
 * 
 * 
 * 3
 * i what to move the robot frod 18 ft. 8in to grab a cub.
 * thin trn baKWRD to scor the cub.
 * 
 */

public class pathPlanner {
    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    private static final Drivetrain drivetrain;

    public pathPlanner(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

    }

    public

    PathPlannerTrajectory DriveForwardOneMeter = PathPlanner.loadPath("DriveForwardOneMeter",
            new PathConstraints(0, 0));

    // This trajectory can then be passed to a path follower such as a
    // PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following

    // Sample the state of the path at 1.2 seconds
    PathPlannerState exampleState = (PathPlannerState) DriveForwardOneMeter.sample(1.2);

    // This will load the file "Example Path Group.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner
            .loadPathGroup("Example Path Group", new PathConstraints(4, 3));

    // This will load the file "Example Path Group.path" and generate it with
    // different path constraints for each segment
    ArrayList<PathPlannerTrajectory> pathGroup2 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Example Path Group",
            new PathConstraints(4, 3),
            new PathConstraints(2, 2),
            new PathConstraints(3, 3));

    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner
            .loadPathGroup("FullAuto", new PathConstraints(4, 3));

    private Map<String, Command> DriveMap;

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drivetrain::getPose, // Pose2d supplier
            drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            drivetrain.mKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and
                                             // Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                             // controller)
            drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            DriveMap,
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            Drivetrain // The drive subsystem. Used to properly set the requirements of path following
                       // commands
    );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

}
