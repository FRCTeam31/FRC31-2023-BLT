package frc.robot.utilities;

import com.pathplanner.lib.auto.PIDConstants;

import prime.models.PidConstants;

public class PathPlannerConverter {
    public static PidConstants fromPPPIDConstants(PIDConstants p) {
        return new PidConstants(p.kP, p.kI, p.kD);
    }

    public static PIDConstants fromPidConstants(PidConstants p) {
        return new PIDConstants(p.kP, p.kI, p.kD_min);
    }

    public static PidConstants toPIDConstants(PIDConstants p) {
        return new PidConstants(p.kP, p.kI, p.kD);
    }

    public static PIDConstants toPPPidConstants(PidConstants p) {
        return new PIDConstants(p.kP, p.kI, p.kD_min);
    }
}
