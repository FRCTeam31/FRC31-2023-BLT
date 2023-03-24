// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;

public class Drivetrain extends SubsystemBase {
    // Build a gyro and a kinematics class for calculating our drive
    boolean mInHighGear = true;
    public WPI_Pigeon2 mGyro;
    public SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            DriveMap.kFrontLeftLocation,
            DriveMap.kRearLeftLocation,
            DriveMap.kRearRightLocation,
            DriveMap.kFrontRightLocation);

    // Swerve Modules
    SwerveModule mFrontLeftModule, mFrontRightModule, mRearLeftModule, mRearRightModule;
    public SwerveModule[] mSwerveModules;
    public SwerveModulePosition[] mSwerveModulePositions = new SwerveModulePosition[4];
    SwerveDriveOdometry mOdometry;
    Field2d mField;

    // PID controllers for autonomous
    public PIDController mAutoTranslationXController;
    public PIDController mAutoTranslationYController;
    public PIDController mAutoRotationController;

    // Log entries for module data
    DoubleLogEntry frontLeftSpeedLog, frontRightSpeedLog, rearLeftSpeedLog, rearRightSpeedLog;
    DoubleLogEntry frontLeftAngleLog, frontRightAngleLog, rearLeftAngleLog, rearRightAngleLog;
    DoubleLogEntry headingLog;
    DoubleArrayLogEntry logCurrentStates, logDesiredStates;

    // State machines
    private SwerveModuleState[] _lastDesiredStates = new SwerveModuleState[4];

    /** Creates a new SwerveDriveTrainSubsystem. */
    public Drivetrain() {
        mGyro = new WPI_Pigeon2(DriveMap.kPigeonId, DriveMap.kCANivoreBusName);

        createSwerveModulesAndOdometry();

        mField = new Field2d();
        SmartDashboard.putData(getName() + "/Field", mField);

        setUpModuleLoggers();
    }

    private void createSwerveModulesAndOdometry() {
        // Set up all the swerve modules
        mFrontLeftModule = new SwerveModule(
                DriveMap.kFrontLeftDrivingMotorId,
                DriveMap.kFrontLeftSteeringMotorId,
                DriveMap.kFrontLeftEncoderId,
                DriveMap.kFrontLeftEncoderOffset,
                DriveMap.kFrontLeftInverted);

        mFrontRightModule = new SwerveModule(
                DriveMap.kFrontRightDrivingMotorId,
                DriveMap.kFrontRightSteeringMotorId,
                DriveMap.kFrontRightEncoderId,
                DriveMap.kFrontRightEncoderOffset,
                DriveMap.kFrontRightInverted);

        mRearLeftModule = new SwerveModule(
                DriveMap.kRearLeftDrivingMotorId,
                DriveMap.kRearLeftSteeringMotorId,
                DriveMap.kRearLeftEncoderId,
                DriveMap.kRearLeftEncoderOffset,
                DriveMap.kRearLeftInverted);

        mRearRightModule = new SwerveModule(
                DriveMap.kRearRightDrivingMotorId,
                DriveMap.kRearRightSteeringMotorId,
                DriveMap.kRearRightEncoderId,
                DriveMap.kRearRightEncoderOffset,
                DriveMap.kRearRightInverted);

        mOdometry = new SwerveDriveOdometry(mKinematics,
                mGyro.getRotation2d(),
                new SwerveModulePosition[] {
                        mFrontLeftModule.getPosition(),
                        mRearLeftModule.getPosition(),
                        mRearRightModule.getPosition(),
                        mFrontRightModule.getPosition(),
                },
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        mSwerveModules = new SwerveModule[] {
                mFrontLeftModule,
                mRearLeftModule,
                mRearRightModule,
                mFrontLeftModule
        };
    }

    public void resetGyro() {
        mGyro.reset();
    }

    public void drive(double strafe, double forward, double rotation, boolean fieldRelative) {
        ChassisSpeeds desiredChassisSpeeds;
        if (fieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(strafe, forward, rotation,
                    mGyro.getRotation2d());
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(strafe, forward, rotation);
        }

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

        mFrontLeftModule.setDesiredState(swerveModuleStates[0], mInHighGear);
        mRearLeftModule.setDesiredState(swerveModuleStates[1], mInHighGear);
        mRearRightModule.setDesiredState(swerveModuleStates[2], mInHighGear);
        mFrontRightModule.setDesiredState(swerveModuleStates[3], mInHighGear);
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        mSwerveModulePositions[0] = mFrontLeftModule.getPosition();
        mSwerveModulePositions[1] = mRearLeftModule.getPosition();
        mSwerveModulePositions[2] = mRearRightModule.getPosition();
        mSwerveModulePositions[3] = mFrontRightModule.getPosition();

        mOdometry.resetPosition(getRotation2d(), mSwerveModulePositions, pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(mGyro.getYaw());
    }

    public double getRotationDegrees() {
        return getRotation2d().getDegrees();
    }

    public void setShift(boolean inHighGear) {
        mInHighGear = inHighGear;
    }

    public void toggleShifter() {
        mInHighGear = !mInHighGear;
    }

    public double getShiftedSpeedCoefficient() {
        return mInHighGear ? DriveMap.kHighGearCoefficient : DriveMap.kLowGearCoefficient;
    }

    public void stopMotors() {
        mFrontLeftModule.stopMotors();
        mRearLeftModule.stopMotors();
        mRearRightModule.stopMotors();
        mFrontRightModule.stopMotors();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        if (_lastDesiredStates.length > 0) {
            builder.addDoubleProperty("Heading", this::getRotationDegrees, null);

            if (_lastDesiredStates[0] != null) {
                builder.addDoubleProperty("Drive - FL Speed", () -> _lastDesiredStates[0].speedMetersPerSecond,
                        null);
                builder.addDoubleProperty("Drive - FL Angle", () -> _lastDesiredStates[0].angle.getDegrees(), null);
            }

            if (_lastDesiredStates[1] != null) {
                builder.addDoubleProperty("Drive - RL Speed", () -> _lastDesiredStates[1].speedMetersPerSecond,
                        null);
                builder.addDoubleProperty("Drive - RL Angle", () -> _lastDesiredStates[1].angle.getDegrees(), null);
            }

            if (_lastDesiredStates[2] != null) {
                builder.addDoubleProperty("Drive - RR Speed", () -> _lastDesiredStates[2].speedMetersPerSecond,
                        null);
                builder.addDoubleProperty("Drive - RR Angle", () -> _lastDesiredStates[2].angle.getDegrees(), null);
            }

            if (_lastDesiredStates[3] != null) {
                builder.addDoubleProperty("Drive - FR Speed", () -> _lastDesiredStates[3].speedMetersPerSecond,
                        null);
                builder.addDoubleProperty("Drive - FR Angle", () -> _lastDesiredStates[3].angle.getDegrees(), null);
            }
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var gyroAngle = mGyro.getRotation2d();
        var robotPose = mOdometry.update(gyroAngle, new SwerveModulePosition[] {
                mFrontLeftModule.getPosition(), mFrontRightModule.getPosition(),
                mRearLeftModule.getPosition(), mRearRightModule.getPosition()

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

        var flState = mFrontLeftModule.getPosition();
        var flDesiredState = _lastDesiredStates[0];
        var rlState = mRearLeftModule.getPosition();
        var rlDesiredState = _lastDesiredStates[0];
        var rrState = mRearRightModule.getPosition();
        var rrDesiredState = _lastDesiredStates[0];
        var frState = mFrontRightModule.getPosition();
        var frDesiredState = _lastDesiredStates[0];

        if (flDesiredState != null) {
            logCurrentStates.append(new double[] {
                    flState.angle.getDegrees(), mFrontLeftModule.getVelocityMetersPerSecond(),
                    rlState.angle.getDegrees(), mRearLeftModule.getVelocityMetersPerSecond(),
                    rrState.angle.getDegrees(), mRearRightModule.getVelocityMetersPerSecond(),
                    frState.angle.getDegrees(), mFrontRightModule.getVelocityMetersPerSecond(),
            });

            logDesiredStates.append(new double[] {
                    flDesiredState.angle.getDegrees(), flDesiredState.speedMetersPerSecond,
                    rlDesiredState.angle.getDegrees(), rlDesiredState.speedMetersPerSecond,
                    rrDesiredState.angle.getDegrees(), rrDesiredState.speedMetersPerSecond,
                    frDesiredState.angle.getDegrees(), frDesiredState.speedMetersPerSecond,
            });
        }
    }

    public void setUpModuleLoggers() {
        var log = DataLogManager.getLog();
        frontLeftSpeedLog = new DoubleLogEntry(log, "/drive/speed/fl");
        frontRightSpeedLog = new DoubleLogEntry(log, "/drive/speed/fr");
        rearLeftSpeedLog = new DoubleLogEntry(log, "/drive/speed/rl");
        rearRightSpeedLog = new DoubleLogEntry(log, "/drive/speed/rr");

        frontLeftAngleLog = new DoubleLogEntry(log, "/drive/angle/fl");
        frontRightAngleLog = new DoubleLogEntry(log, "/drive/angle/fr");
        rearLeftAngleLog = new DoubleLogEntry(log, "/drive/angle/rl");
        rearRightAngleLog = new DoubleLogEntry(log, "/drive/angle/rr");
        headingLog = new DoubleLogEntry(log, "/heading");

        logCurrentStates = new DoubleArrayLogEntry(DataLogManager.getLog(), "/drive/currentStates");
        logDesiredStates = new DoubleArrayLogEntry(DataLogManager.getLog(), "/drive/desiredStates");
    }
}
