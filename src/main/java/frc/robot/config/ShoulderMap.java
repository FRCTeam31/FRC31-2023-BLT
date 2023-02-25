package frc.robot.config;

import prime.models.PidConstants;

public class ShoulderMap {
    // CAN ids
    public static final int kShoulder1Id = 31;
    public static final int kShoulder2Id = 32;
    public static final int kEncoderId = 30;

    // PID
    public static final PidConstants AnglePid = new PidConstants(1);
}
