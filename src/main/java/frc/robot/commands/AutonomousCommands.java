package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class AutonomousCommands {
    /**
     * Gets the HashMap of events that can be triggered from a PP trajectory
     * 
     * @param shoulder
     * @param forearm
     * @param wrist
     */
    public static HashMap<String, Command> getFullRobotEventMap(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.putAll(ShoulderCommands.getEvents(shoulder));
        // eventMap.putAll(ForearmCommands.getEvents(forearm));
        // eventMap.putAll(WristCommands.getEvents(wrist));
        // eventMap.put("waitOneSecond", Commands.waitSeconds(1));

        return eventMap;
    }

    /***
     * Pulls the autonomous paths from PathPlanner
     * Change the string in line 27 to the name of the path you want to use.
     * 
     * @param drivetrain
     * @return
     */
    public static Command getAutonomousCommand(Drivetrain drivetrain, Shoulder shoulder, Forearm forearm, Wrist wrist) {
        var traj = PathPlanner.loadPath("DriveForwardOneMeter",
                new PathConstraints(DriveMap.kDriveMaxSpeedMetersPerSecond, 2));
        // List<PathPlannerTrajectory> fullAuto =
        // PathPlanner.loadPathGroup("DriveForwardOneMeter",
        // DriveMap.kDriveMaxSpeedMetersPerSecond, 2);
        // ArrayList<PathPlannerTrajectory> fullAutoArrayList = new
        // ArrayList<>(fullAuto);

        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        // drivetrain::getPose,
        // drivetrain::resetOdometry,
        // // Tune with instructions at:
        // //
        // //
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html#constructing-a-holonomic-drive-controller
        // new PIDConstants(0, 0, 0),
        // new PIDConstants(0, 0, 0),
        // drivetrain::drive,
        // getFullRobotEventMap(shoulder, forearm, wrist),
        // drivetrain);

        // return autoBuilder.fullAuto(fullAutoArrayList);

        // SwerveAutoBuilder autoBuilder2 = new SwerveAutoBuilder(drivetrain::getPose,
        // drivetrain::resetOdometry,
        // drivetrain.Kinematics,
        // new PIDConstants(32.8, 0, 0),
        // new PIDConstants(0, 0, 0),
        // drivetrain::drive,
        // getFullRobotEventMap(shoulder, forearm, wrist),
        // true,
        // drivetrain);

        // return autoBuilder2.fullAuto(fullAutoArrayList);

        return new PPSwerveControllerCommand(
                traj,
                drivetrain::getPose, // Pose supplier
                drivetrain.Kinematics, // SwerveDriveKinematics
                new PIDController(1.28, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will
                                               // only
                                               // use feedforwards.
                new PIDController(0.03, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0.075, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0
                                                // will only use feedforwards.
                drivetrain::drive, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                       // Optional, defaults to true
                drivetrain // Requires this drive subsystem
        );
    }
}