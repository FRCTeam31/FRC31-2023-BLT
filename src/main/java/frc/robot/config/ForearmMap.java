package frc.robot.config;

import prime.models.PidConstants;

public class ForearmMap {
    // CAN IDs
    public static final int kForearmMotor1Id = 23;
    public static final int kForearmMotor2Id = 24;

    // PID
    public static final PidConstants AnglePid = new PidConstants(0.04);

    // Constants
    public static final double kOpenLoopRampRate = 1.00;
}
