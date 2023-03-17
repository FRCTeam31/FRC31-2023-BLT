package frc.robot.config;

import prime.models.PidConstants;

public class ForearmMap {
    // CAN IDs
    public static final int kForearmMotor1Id = 23;

    // PID
    public static final PidConstants PushAndPullPIDConstants = new PidConstants(0.04);

    // Constants
    public static final double kOpenLoopRampRate = 1.00;
}
