// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;

public class Drivetrain extends SubsystemBase {

    public SwerveModulePosition[] swerveModules = new SwerveModulePosition[4];

    // Default PID values for steering each module and driving each module
    private final Field2d mField = new Field2d();

    // Initialize "locations" of each wheel in terms of x, y translation in meters
    // from the origin (middle of the robot)
    double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
    double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
    final Translation2d frontLeftLocation = new Translation2d(-halfTrackWidth, halfWheelBase);
    final Translation2d frontRightLocation = new Translation2d(halfTrackWidth, halfWheelBase);
    final Translation2d rearLeftLocation = new Translation2d(-halfTrackWidth, -halfWheelBase);
    final Translation2d rearRightLocation = new Translation2d(halfTrackWidth, -halfWheelBase);

    // Build a gyro and a kinematics class for our drive
    final WPI_Pigeon2 mGyro = new WPI_Pigeon2(DriveMap.kPigeonId, DriveMap.kCANivoreBusName);
    public final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            rearLeftLocation,
            rearRightLocation,
            frontRightLocation);

    // Swerve Modules
    SwerveModule FrontLeftSwerveModule;
    SwerveModule RearLeftSwerveModule;
    SwerveModule RearRightSwerveModule;
    SwerveModule FrontRightSwerveModule;

    SwerveDriveOdometry mOdometry;

    private DoubleLogEntry frontLeftSpeedLog, frontRightSpeedLog, rearLeftSpeedLog, rearRightSpeedLog;
    private DoubleLogEntry frontLeftAngleLog, frontRightAngleLog, rearLeftAngleLog, rearRightAngleLog;
    private DoubleLogEntry headingLog;

    // State machines
    private SwerveModuleState[] _lastDesiredStates = new SwerveModuleState[4];

    /** Creates a new SwerveDriveTrainSubsystem. */
    public Drivetrain(SwerveModule FrontLeftSwerveModule, SwerveModule FrontRightSwerveModule,
            SwerveModule RearLeftSwerveModule, SwerveModule RearRightSwerveModule) {
        SmartDashboard.putData("Field", mField);
        mGyro.reset();

        this.mOdometry = new SwerveDriveOdometry(mKinematics,
                mGyro.getRotation2d(),
                new SwerveModulePosition[] {
                        FrontLeftSwerveModule.getPosition(),
                        RearLeftSwerveModule.getPosition(),
                        RearRightSwerveModule.getPosition(),
                        FrontRightSwerveModule.getPosition(),
                },
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        this.FrontLeftSwerveModule = FrontLeftSwerveModule;
        this.FrontRightSwerveModule = FrontRightSwerveModule;
        this.RearLeftSwerveModule = RearLeftSwerveModule;
        this.RearRightSwerveModule = RearRightSwerveModule;

        var log = DataLogManager.getLog();
        frontLeftSpeedLog = new DoubleLogEntry(log, "/drive/speed/fl");
        frontRightSpeedLog = new DoubleLogEntry(log, "/drive/speed/fr");
        rearLeftSpeedLog = new DoubleLogEntry(log, "/drive/speed/rl");
        rearRightSpeedLog = new DoubleLogEntry(log, "/drive/speed/rr");

        frontLeftAngleLog = new DoubleLogEntry(log, "/drive/angle/fl");
        frontRightAngleLog = new DoubleLogEntry(log, "/drive/angle/fr");
        rearLeftAngleLog = new DoubleLogEntry(log, "/drive/angle/rl");
        rearRightAngleLog = new DoubleLogEntry(log, "/drive/angle/rr");
        var robotHeadinglog = DataLogManager.getLog();
        headingLog = new DoubleLogEntry(log, "/heading");

    }

    public void resetGyro() {
        mGyro.reset();
    }

    public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
        var desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(strafe, forward, rotation, mGyro.getRotation2d())
                : new ChassisSpeeds(strafe, forward, rotation);

        drive(desiredChassisSpeeds);
    }

    public void drive(ChassisSpeeds desiredChassisSpeeds) {
        var swerveModuleStates = mKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.kDriveMaxSpeedMetersPerSecond);

        drive(swerveModuleStates);
    }

    public void drive(SwerveModuleState[] swerveModuleStates) {
        _lastDesiredStates = swerveModuleStates;
        frontLeftSpeedLog.append(swerveModuleStates[0].speedMetersPerSecond);
        frontLeftAngleLog.append(swerveModuleStates[0].angle.getDegrees());
        rearLeftSpeedLog.append(swerveModuleStates[1].speedMetersPerSecond);
        rearLeftAngleLog.append(swerveModuleStates[1].angle.getDegrees());
        rearRightSpeedLog.append(swerveModuleStates[2].speedMetersPerSecond);
        rearRightAngleLog.append(swerveModuleStates[2].angle.getDegrees());
        frontRightSpeedLog.append(swerveModuleStates[3].speedMetersPerSecond);
        frontRightAngleLog.append(swerveModuleStates[3].angle.getDegrees());

        headingLog.append(Math.toDegrees(mGyro.getYaw()));

        FrontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        RearLeftSwerveModule.setDesiredState(swerveModuleStates[1]);
        RearRightSwerveModule.setDesiredState(swerveModuleStates[2]);
        FrontRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveModules[0] = FrontLeftSwerveModule.getPosition();
        swerveModules[1] = RearLeftSwerveModule.getPosition();
        swerveModules[2] = RearRightSwerveModule.getPosition();
        swerveModules[3] = FrontRightSwerveModule.getPosition();

        mOdometry.resetPosition(getRotation2d(), swerveModules, pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(mGyro.getYaw());
    }

    public double getRotationDegrees() {
        return getRotation2d().getDegrees();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("Drivetrain gyro angle", getRotationDegrees());

        if (_lastDesiredStates.length > 0) {
            // if (_lastDesiredStates[0] != null) {
            // SmartDashboard.putNumber("Drive - FL Speed", () ->
            // _lastDesiredStates[0].speedMetersPerSecond, null);
            // SmartDashboard.putNumber("Drive - FL Angle", () ->
            // _lastDesiredStates[0].angle.getDegrees(), null);
            // }

            // if (_lastDesiredStates[1] != null) {
            // SmartDashboard.putNumber("Drive - RL Speed", () ->
            // _lastDesiredStates[1].speedMetersPerSecond, null);
            // SmartDashboard.putNumber("Drive - RL Angle", () ->
            // _lastDesiredStates[1].angle.getDegrees(), null);
            // }

            // if (_lastDesiredStates[2] != null) {
            // SmartDashboard.putNumber("Drive - RR Speed", () ->
            // _lastDesiredStates[2].speedMetersPerSecond, null);
            // SmartDashboard.putNumber("Drive - RR Angle", () ->
            // _lastDesiredStates[2].angle.getDegrees(), null);
            // }

            // if (_lastDesiredStates[3] != null) {
            // SmartDashboard.putNumber("Drive - FR Speed", () ->
            // _lastDesiredStates[3].speedMetersPerSecond, null);
            // SmartDashboard.putNumber("Drive - FR Angle", () ->
            // _lastDesiredStates[3].angle.getDegrees(), null);
            // }
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var gyroAngle = mGyro.getRotation2d();
        var robotPose = mOdometry.update(gyroAngle, new SwerveModulePosition[] {
                FrontLeftSwerveModule.getPosition(), FrontRightSwerveModule.getPosition(),
                RearLeftSwerveModule.getPosition(), RearRightSwerveModule.getPosition()

        });

        mField.setRobotPose(robotPose);

        if (_lastDesiredStates[0] != null) {
            SmartDashboard.putNumber("Drive - FL Speed", _lastDesiredStates[0].speedMetersPerSecond);
            SmartDashboard.putNumber("Drive - FL Angle", _lastDesiredStates[0].angle.getDegrees());
            SmartDashboard.putNumber("Drive - RL Speed", _lastDesiredStates[1].speedMetersPerSecond);
            SmartDashboard.putNumber("Drive - RL Angle", _lastDesiredStates[1].angle.getDegrees());
            SmartDashboard.putNumber("Drive - RR Speed", _lastDesiredStates[2].speedMetersPerSecond);
            SmartDashboard.putNumber("Drive - RR Angle", _lastDesiredStates[2].angle.getDegrees());
            SmartDashboard.putNumber("Drive - FR Speed", _lastDesiredStates[3].speedMetersPerSecond);
            SmartDashboard.putNumber("Drive - FR Angle", _lastDesiredStates[3].angle.getDegrees());
        }
    }
}
