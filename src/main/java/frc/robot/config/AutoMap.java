package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PathPlannerConverter;

public class AutoMap {
    public static final String kTranslatePidConstantsName = "Auto Translation PID Constants";
    public static PIDConstants kTranslatePidConstants = new PIDConstants(20, 0, 0);
    public static final String kRotatePidConstantsName = "Auto Rotation PID Constants";
    public static PIDConstants kRotatePidConstants = new PIDConstants(1, 0, 0);

    AutoMap() {
        SmartDashboard.putData(
                kTranslatePidConstantsName,
                PathPlannerConverter.fromPPPIDConstants(kTranslatePidConstants));

        SmartDashboard.putData(
                kRotatePidConstantsName,
                PathPlannerConverter.fromPPPIDConstants(kRotatePidConstants));
    }
}
