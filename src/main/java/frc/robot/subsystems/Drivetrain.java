// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
            frontRightLocation,
            rearLeftLocation,
            rearRightLocation);

    // Swerve Modules
    SwerveModule FrontLeftSwerveModule;
    SwerveModule FrontRightSwerveModule;
    SwerveModule RearLeftSwerveModule;
    SwerveModule RearRightSwerveModule;

    SwerveDriveOdometry mOdometry;

    /** Creates a new SwerveDriveTrainSubsystem. */
    public Drivetrain(SwerveModule FrontLeftSwerveModule, SwerveModule FrontRightSwerveModule,
            SwerveModule RearLeftSwerveModule, SwerveModule RearRightSwerveModule) {
        SmartDashboard.putData("Field", mField);
        mGyro.reset();

        this.mOdometry = new SwerveDriveOdometry(mKinematics,
                mGyro.getRotation2d(),
                new SwerveModulePosition[] {
                        FrontLeftSwerveModule.getPosition(),
                        FrontRightSwerveModule.getPosition(),
                        RearLeftSwerveModule.getPosition(),
                        RearRightSwerveModule.getPosition(),
                },
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        this.FrontLeftSwerveModule = FrontLeftSwerveModule;
        this.FrontRightSwerveModule = FrontRightSwerveModule;
        this.RearLeftSwerveModule = RearLeftSwerveModule;
        this.RearRightSwerveModule = RearRightSwerveModule;

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var gyroAngle = mGyro.getRotation2d();
        SmartDashboard.putNumber("Drivetrain gyro angle", gyroAngle.getDegrees());

        var robotPose = mOdometry.update(gyroAngle, new SwerveModulePosition[] {
                FrontLeftSwerveModule.getPosition(), FrontRightSwerveModule.getPosition(),
                RearLeftSwerveModule.getPosition(), RearRightSwerveModule.getPosition()
        });

        mField.setRobotPose(robotPose);

        if (swerveModules[0] != null) {
            SmartDashboard.putNumber("FrontLeftCurrentSpeed", swerveModules[0].distanceMeters);
            SmartDashboard.putNumber("FrontLeftCurrentAngle", swerveModules[0].angle.getDegrees());
        }

        if (swerveModules[1] != null) {
            SmartDashboard.putNumber("FrontRightCurrentSpeed", swerveModules[1].distanceMeters);
            SmartDashboard.putNumber("FrontRightCurrentAngle", swerveModules[1].angle.getDegrees());
        }

        if (swerveModules[2] != null) {
            SmartDashboard.putNumber("RearLeftCurrentSpeed", swerveModules[2].distanceMeters);
            SmartDashboard.putNumber("RearLeftCurrentAngle", swerveModules[2].angle.getDegrees());
        }

        if (swerveModules[3] != null) {
            SmartDashboard.putNumber("RearRightCurrentSpeed", swerveModules[3].distanceMeters);
            SmartDashboard.putNumber("RearLeftCurrentAngle", swerveModules[3].angle.getDegrees());
        }
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
        FrontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        FrontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
        RearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
        RearRightSwerveModule.setDesiredState(swerveModuleStates[3]);

    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {

        swerveModules[0] = FrontLeftSwerveModule.getPosition();
        swerveModules[1] = FrontRightSwerveModule.getPosition();
        swerveModules[2] = RearLeftSwerveModule.getPosition();
        swerveModules[3] = RearRightSwerveModule.getPosition();

        mOdometry.resetPosition(getRotation2d(), swerveModules, pose);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(mGyro.getYaw());
    }

}
