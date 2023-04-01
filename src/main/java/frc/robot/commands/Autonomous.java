package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Drivetrain;

public class Autonomous {
    // public Command getAutonomousCommand(){
    // ArrayList<PathPlannerTrajectory> pathGroup =
    // (ArrayList)PathPlanner.loadPathGroup("DriveForwardOneMeter", 3, 4)
    // }

    /***
     * Pulls the autonomous paths from PathPlanner
     * Change the string in line 27 to the name of the path you want to use.
     * 
     * @param drivetrain
     * @return
     */
    public Command getAutonomousCommand(Drivetrain drivetrain) {
        List<PathPlannerTrajectory> fullAuto = PathPlanner.loadPathGroup("DriveForwardOneMeter", 3, 4);
        ArrayList<PathPlannerTrajectory> fullAutoArrayList = new ArrayList<>(fullAuto);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("test event", new PrintCommand("test event"));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::resetOdometry,
                new PIDConstants(0, 0, 0),
                new PIDConstants(0, 0, 0),
                drivetrain::drive,
                eventMap,
                drivetrain);

        Command fullAutoCommand = autoBuilder.fullAuto(fullAutoArrayList);
        return fullAutoCommand;
    }
}