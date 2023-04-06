package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class Autonomous {
    /***
     * Pulls the autonomous paths from PathPlanner
     * Change the string in line 27 to the name of the path you want to use.
     * 
     * @param drivetrain
     * @return
     * 
     */
    public Command getAutonomousCommand(Drivetrain drivetrain, Shoulder shoulder, Forearm forearm, Wrist wrist) {
        List<PathPlannerTrajectory> fullAuto = PathPlanner.loadPathGroup("DriveForwardOneMeter", 0.1, 0.1);
        ArrayList<PathPlannerTrajectory> fullAutoArrayList = new ArrayList<>(fullAuto);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.putAll(ShoulderCommands.getEvents(shoulder));
        eventMap.putAll(ForearmCommands.getEvents(forearm));
        eventMap.putAll(WristCommands.getEvents(wrist));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                new PIDConstants(0, 0, 0),
                new PIDConstants(0, 0, 0),
                drivetrain::drive,
                eventMap,
                drivetrain);

        return autoBuilder.fullAuto(fullAutoArrayList);
    }
}