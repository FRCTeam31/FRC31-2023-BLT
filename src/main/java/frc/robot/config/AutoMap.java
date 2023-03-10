package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;

public class AutoMap {
    public static PIDConstants kTranslatePidConstants = new PIDConstants(20, 0, 0);
    public static PIDConstants kRotatePidConstants = new PIDConstants(1, 0, 0);
}
