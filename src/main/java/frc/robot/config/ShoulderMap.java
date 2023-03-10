package frc.robot.config;

import prime.models.PidConstants;

public class ShoulderMap {
    // CAN ids
    public static final int kShoulder1Id = 23;
    public static final int kShoulder2Id = 24;
    public static final int kEncoderId = 20;

    // PID
    public static final PidConstants AnglePid = new PidConstants(0.04);

    // constants
    public static final double kOpenLoopRampRate = 1.00;
}
