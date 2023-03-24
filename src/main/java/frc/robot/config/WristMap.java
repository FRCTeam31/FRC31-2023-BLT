package frc.robot.config;

public class WristMap {
    // CAN ids
    public static final int kWrist1Id = 22;
    public static final int kWristActuatorId = 0;

    // DIO channel
    public static final int kWristHallSensorChannel = 9;

    // Motor speeds
    public static final double kIntakeConeSpeed = -0.6;
    public static final double kEjectConeSpeed = 0.6;
    public static final double kIntakeCubeSpeed = 0.4;
    public static final double kEjectCubeSpeed = -0.6;
    public static final double triggerDeadBand = 0.4;
}
