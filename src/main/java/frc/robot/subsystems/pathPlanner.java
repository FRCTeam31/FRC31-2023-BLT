package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    // public pathPlanner(SubsystemBase subsystemBase) {
    // pathPlanner.SubsystemBase = subsystemBase;
    // }

    public

    PathPlannerTrajectory AdvisPath = PathPlanner.loadPath("AdvisPath",
            new PathConstraints(5, 5));

    // This trajectory can then be passed to a path follower such as a
    // PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following

    // Sample the state of the path at 1.2 seconds
    PathPlannerState exampleState = (PathPlannerState) AdvisPath.sample(1.2);

    // This will load the file "Example Path Group.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner
            .loadPathGroup("AdvisPath", new PathConstraints(4, 3));

    // This will load the file "Example Path Group.path" and generate it with
    // different path constraints for each segment
    ArrayList<PathPlannerTrajectory> pathGroup2 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "AdvisPath",
            new PathConstraints(4, 3),
            new PathConstraints(2, 2),
            new PathConstraints(3, 3));

    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("AdvisPath", new PathConstraints(4, 3));

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();

    FollowPathWithEvents command = new FollowPathWithEvents(
            getPathFollowingCommand(AdvisPath),
            AdvisPath.getMarkers(),
            eventMap);

    private Command getPathFollowingCommand(PathPlannerTrajectory AdvisPath) {
        return null;
    }

    // Simple path without holonomic rotation. Stationary start/end. Max velocity of
    // 4 m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
    );

    // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
    // m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj2 = PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position,
                                                                                                              // heading(direction
                                                                                                              // of
                                                                                                              // travel),
                                                                                                              // holonomic
                                                                                                              // rotation
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)) // position,
                                                                                                                // heading(direction
                                                                                                                // of
                                                                                                                // travel),
                                                                                                                // holonomic
                                                                                                                // rotation
    );

    // More complex path with holonomic rotation. Non-zero starting velocity of 2
    // m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
    PathPlannerTrajectory traj3 = PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2), // position,
                                                                                                                 // heading(direction
                                                                                                                 // of
                                                                                                                 // travel),
                                                                                                                 // holonomic
                                                                                                                 // rotation,
                                                                                                                 // velocity
                                                                                                                 // override
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)), // position,
                                                                                                                 // heading(direction
                                                                                                                 // of
                                                                                                                 // travel),
                                                                                                                 // holonomic
                                                                                                                 // rotation
            new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position,
                                                                                                               // heading(direction
                                                                                                               // of
                                                                                                               // travel),
                                                                                                               // holonomic
                                                                                                               // rotation
    );

}
