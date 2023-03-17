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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;

public class Drivetrain extends SubsystemBase {

    public SwerveModulePosition[] swerveModules = new SwerveModulePosition[4];

    // Default PID values for steering each module and driving each module
    Field2d mField = new Field2d();

    // Initialize "locations" of each wheel in terms of x, y translation in meters
    // from the origin (middle of the robot)
    double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
    double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;
    final Translation2d frontLeftLocation = new Translation2d(-halfTrackWidth, halfWheelBase);
    final Translation2d frontRightLocation = new Translation2d(halfTrackWidth, halfWheelBase);
    final Translation2d rearLeftLocation = new Translation2d(-halfTrackWidth, -halfWheelBase);
    final Translation2d rearRightLocation = new Translation2d(halfTrackWidth, -halfWheelBase);

    // Swerve Modules
    SwerveModule FrontLeftSwerveModule;
    SwerveModule FrontRightSwerveModule;
    SwerveModule RearLeftSwerveModule;
    SwerveModule RearRightSwerveModule;
    SwerveDriveOdometry mOdometry;

    // Build a gyro and a kinematics class for our drive
    WPI_Pigeon2 mGyro = new WPI_Pigeon2(DriveMap.kPigeonId, DriveMap.kCANivoreBusName);
    SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            rearLeftLocation,
            rearRightLocation);

    boolean mInHighGear = false;

    // State machines
    private SwerveModuleState[] _lastDesiredStates = null;

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

    public void setShift(boolean inHighGear) {
        mInHighGear = inHighGear;
    }

    public void toggleShifter() {
        mInHighGear = !mInHighGear;
    }

    public double getShiftedSpeedCoefficient() {
        return mInHighGear ? DriveMap.kHighGearCoefficient : DriveMap.kLowGearCoefficient;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drivetrain gyro angle", this::getRotationDegrees, null);

        builder.addDoubleProperty("Drive - FL Speed", () -> _lastDesiredStates[0].speedMetersPerSecond, null);
        builder.addDoubleProperty("Drive - FL Angle", () -> _lastDesiredStates[0].angle.getDegrees(), null);

        builder.addDoubleProperty("Drive - RL Speed", () -> _lastDesiredStates[1].speedMetersPerSecond, null);
        builder.addDoubleProperty("Drive - RL Angle", () -> _lastDesiredStates[1].angle.getDegrees(), null);

        builder.addDoubleProperty("Drive - RR Speed", () -> _lastDesiredStates[2].speedMetersPerSecond, null);
        builder.addDoubleProperty("Drive - RR Angle", () -> _lastDesiredStates[2].angle.getDegrees(), null);

        builder.addDoubleProperty("Drive - FR Speed", () -> _lastDesiredStates[3].speedMetersPerSecond, null);
        builder.addDoubleProperty("Drive - FR Angle", () -> _lastDesiredStates[3].angle.getDegrees(), null);
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

        var flState = FrontLeftSwerveModule.getPosition();
        var flDesiredState = _lastDesiredStates[0];
        var rlState = RearLeftSwerveModule.getPosition();
        var rlDesiredState = _lastDesiredStates[1];
        var rrState = RearRightSwerveModule.getPosition();
        var rrDesiredState = _lastDesiredStates[2];
        var frState = FrontRightSwerveModule.getPosition();
        var frDesiredState = _lastDesiredStates[3];

        var currentStates = new double[] {
            flState.getDegrees(), FrontLeftSwerveModule.getRawSpeed(), 
        }
    }
}
