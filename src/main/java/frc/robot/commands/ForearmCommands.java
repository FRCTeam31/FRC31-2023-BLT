package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Forearm;

public class ForearmCommands {

    /***
     * Extends the Forearm.
     * 
     * @param forearm
     * @return
     */
    public static Command extendForearm(Forearm forearm) {
        return Commands.runOnce(() -> {
            forearm.extendForearmSolenoid(true);
        }, forearm);
    }

    /***
     * Retracts the Forearm.
     * 
     * @param forearm
     * @return
     */
    public static Command retractForearm(Forearm forearm) {
        return Commands.runOnce(() -> {
            forearm.extendForearmSolenoid(false);
        }, forearm);
    }

}
