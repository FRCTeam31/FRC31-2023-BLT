package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;

public class AutoMap {
    public static final String kTranslatePidConstantsName = "Auto Translation PID Constants";
    public static PIDConstants kTranslatePidConstants = new PIDConstants(0.0001, 0, 0.00);
    public static final String kRotatePidConstantsName = "Auto Rotation PID Constants";
    public static PIDConstants kRotatePidConstants = new PIDConstants(0.8, 0, 0);
}
