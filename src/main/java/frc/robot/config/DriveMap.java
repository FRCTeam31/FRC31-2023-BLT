package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import prime.models.PidConstants;
import prime.utilities.CTREConverter;

public class DriveMap {
    // Physical measurements
    public static final double kRobotTrackWidthMeters = 0.4445;
    public static final double kRobotWheelBaseMeters = 0.6477;
    public static final double kRobotWheelBaseCircumferenceMeters = 2.46788;
    public static final double kDriveWheelDiameterMeters = 0.102;
    public static final byte kDriveMotorOutputTeeth = 13;
    public static final byte kDriveMotorDrivenGearTeeth = 42;

    // Calculated values
    public static final double kDriveGearRatio = kDriveMotorDrivenGearTeeth / kDriveMotorOutputTeeth;
    public static final double kDriveWheelCircumference = Math.PI * kDriveWheelDiameterMeters;

    // Locations of each wheel in terms of x, y translation in meters from the
    // middle of the robot
    static final double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
    static final double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
    public static final Translation2d kFrontLeftLocation = new Translation2d(-halfTrackWidth, halfWheelBase);
    public static final Translation2d kFrontRightLocation = new Translation2d(halfTrackWidth, halfWheelBase);
    public static final Translation2d kRearLeftLocation = new Translation2d(-halfTrackWidth, -halfWheelBase);
    public static final Translation2d kRearRightLocation = new Translation2d(halfTrackWidth, -halfWheelBase);

    // Measured SysId values
    public static final double driveKs = -0.13939;
    public static final double driveKv = 0.029115;
    public static final double driveKa = 0.0050108;
    public static final String kDrivePidConstantsName = "SwerveModule drive PID Constants";
    public static PidConstants kDrivePidConstants = new PidConstants(0.016983, 1);
    public static final String kSteeringPidConstantsName = "SwerveModule steering PID Constants";
    public static PidConstants kSteeringPidConstants = new PidConstants(0.2);

    // Pigeon
    public static final int kPigeonId = 10;
    public static final String kCANivoreBusName = "Team31CANivore";

    // FR
    public static int kFrontRightSteeringMotorId = 1;
    public static int kFrontRightDrivingMotorId = 2;
    public static int kFrontRightEncoderId = 11;
    public static short kFrontRightEncoderOffset = 49 - 90;
    public static boolean kFrontRightInverted = true;

    // FL
    public static int kFrontLeftSteeringMotorId = 3;
    public static int kFrontLeftDrivingMotorId = 4;
    public static int kFrontLeftEncoderId = 12;
    public static short kFrontLeftEncoderOffset = 248 - 90;
    public static boolean kFrontLeftInverted = false;

    // RR
    public static int kRearRightSteeringMotorId = 7;
    public static int kRearRightDrivingMotorId = 8;
    public static int kRearRightEncoderId = 14;
    public static short kRearRightEncoderOffset = 43 - 90;
    public static boolean kRearRightInverted = true;

    // RL
    public static int kRearLeftSteeringMotorId = 5;
    public static int kRearLeftDrivingMotorId = 6;
    public static int kRearLeftEncoderId = 13;
    public static short kRearLeftEncoderOffset = 244 - 90;
    public static boolean kRearLeftInverted = false;

    // Gear ratios
    public static byte driveMotorOutputTeeth = 13;
    public static byte driveMotorDriveGearTeeth = 42;
    public static int falconTotalSensorUnits = 2048;
    public static final double kDriveMaxSpeedMetersPerSecond = CTREConverter.falconToMPS(22000,
            kDriveWheelCircumference, kDriveGearRatio); // 16.2ft

    // per second in meters per second
    public static final double kDriveMaxAngularSpeed = (DriveMap.kRobotWheelBaseCircumferenceMeters / 2)
            / kDriveMaxSpeedMetersPerSecond;
    public static final double kHighGearCoefficient = 0.7;
    public static final double kLowGearCoefficient = 0.15;

    // Drive Auton PID values
    public static final double kAutoTranslationPID_kP = 2.01;
    public static final double kAutoTranslationPID_kI = 0;
    public static final double kAutoTranslationPID_kD = 1;

    public static final double kAutoRotationPID_kP = 1;
    public static final double kAutoRotationPID_kI = 0;
    public static final double kAutoRotationPID_kD = 0;
}
