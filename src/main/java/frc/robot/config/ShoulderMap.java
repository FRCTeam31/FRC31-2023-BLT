package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import prime.models.PidConstants;

public class ShoulderMap {
    // CAN ids
    public static final int kShoulder1Id = 23;
    public static final int kShoulder2Id = 24;
    public static final int kEncoderId = 20;

    // PID
    public static final String kSprocketPidName = "Shoulder PID constants";
    public static final PidConstants kSprocketPid = new PidConstants(0.04);

    // Constants
    public static final double kOpenLoopRampRate = 1.00;
    public static final double kMaxAngle = 200;
    public static final double kMinAngle = 150;

    public ShoulderMap() {
        SmartDashboard.putData(kSprocketPidName, kSprocketPid);
    }
}
