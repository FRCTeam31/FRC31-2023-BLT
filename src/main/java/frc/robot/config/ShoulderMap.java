package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import prime.models.PidConstants;

public class ShoulderMap {
    // CAN ids
    public static final int kShoulder1Id = 31;
    public static final int kShoulder2Id = 32;
    public static final int kEncoderId = 30;

    // PID
    public static final String kSprocketPidName = "Shoulder PID constants";
    public static final PidConstants kSprocketPid = new PidConstants(1);

    public ShoulderMap() {
        SmartDashboard.putData(kSprocketPidName, kSprocketPid);
    }
}
