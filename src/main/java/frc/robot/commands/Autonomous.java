package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Forearm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

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
    public Command getAutonomousCommand(Drivetrain drivetrain, Shoulder shoulder, Forearm forearm, Wrist wrist) {
        List<PathPlannerTrajectory> fullAuto = PathPlanner.loadPathGroup("DriveForwardOneMeter", 1, 1);
        ArrayList<PathPlannerTrajectory> fullAutoArrayList = new ArrayList<>(fullAuto);

        HashMap<String, Command> shoulderEvents = ShoulderCommands.getEvents(shoulder);
        HashMap<String, Command> forearmEvent = ForearmCommands.getEvents(forearm);
        HashMap<String, Command> wristEvents = WristCommands.getEvents(wrist);
        HashMap<String, Command> eventMap = new HashMap<>() {

            @Override
            public void putAll(Map<? extends String, ? extends Command> m) {
                super.putAll(shoulderEvents);
                super.putAll(forearmEvent);
                super.putAll(wristEvents);
            }

        };

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