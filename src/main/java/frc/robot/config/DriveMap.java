package frc.robot.config;

import prime.models.PidConstants;

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
  
  // Measured SysId values
  public static final double driveKs = -0.13939;
  public static final double driveKv = 0.029115;
  public static final double driveKa = 0.0050108;
  public static final PidConstants kDrivePidConstants = new PidConstants(0.0016983);
  public static final PidConstants kSteeringPidConstants = new PidConstants(0.2);

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
  public static final double kDriveMaxSpeedMetersPerSecond = 4.938; // 16.2ft per second in meters per second
  public static final double kDriveMaxAngularSpeed = DriveMap.kRobotWheelBaseCircumferenceMeters
      / kDriveMaxSpeedMetersPerSecond;
  public static final double kHighGearCoefficient = 0.7;
  public static final double kLowGearCoefficient = 0.15;
}

