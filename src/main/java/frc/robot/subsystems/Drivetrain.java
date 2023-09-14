package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    // Gyro and Kinematics
    public WPI_Pigeon2 Gyro;
    public SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
            // in CCW order from FL to FR
            DriveMap.kFrontLeftLocation,
            DriveMap.kRearLeftLocation,
            DriveMap.kRearRightLocation,
            DriveMap.kFrontRightLocation);
    private boolean _inHighGear = true;

    // Swerve Modules, in CCW order from FL to FR
    SwerveModule mFrontLeftModule, mRearLeftModule, mRearRightModule, mFrontRightModule;
    public SwerveModule[] mSwerveModules;
    public SwerveModulePosition[] mSwerveModulePositions = new SwerveModulePosition[4];

    // Odometry
    SwerveDriveOdometry mOdometry;
    Field2d mField;

    public Drivetrain() {
        setName("Drivetrain");
        Gyro = new WPI_Pigeon2(DriveMap.kPigeonId, DriveMap.kCANivoreBusName);

        createSwerveModulesAndOdometry();

        mField = new Field2d();
        SmartDashboard.putData(getName() + "/Field", mField);
    }

    /**
     * Creates the swerve modules and starts odometry
     */
    private void createSwerveModulesAndOdometry() {
        mFrontLeftModule = new SwerveModule(
                DriveMap.kFrontLeftDrivingMotorId,
                DriveMap.kFrontLeftSteeringMotorId,
                DriveMap.kFrontLeftEncoderId,
                DriveMap.kFrontLeftEncoderOffset,
                DriveMap.kFrontLeftInverted);
        mFrontLeftModule.register();

        mFrontRightModule = new SwerveModule(
                DriveMap.kFrontRightDrivingMotorId,
                DriveMap.kFrontRightSteeringMotorId,
                DriveMap.kFrontRightEncoderId,
                DriveMap.kFrontRightEncoderOffset,
                DriveMap.kFrontRightInverted);
        mFrontRightModule.register();

        mRearLeftModule = new SwerveModule(
                DriveMap.kRearLeftDrivingMotorId,
                DriveMap.kRearLeftSteeringMotorId,
                DriveMap.kRearLeftEncoderId,
                DriveMap.kRearLeftEncoderOffset,
                DriveMap.kRearLeftInverted);
        mRearLeftModule.register();

        mRearRightModule = new SwerveModule(
                DriveMap.kRearRightDrivingMotorId,
                DriveMap.kRearRightSteeringMotorId,
                DriveMap.kRearRightEncoderId,
                DriveMap.kRearRightEncoderOffset,
                DriveMap.kRearRightInverted);
        mRearRightModule.register();

        mOdometry = new SwerveDriveOdometry(Kinematics,
                Gyro.getRotation2d(),
                new SwerveModulePosition[] { // in CCW order from FL to FR
                        mFrontLeftModule.getPosition(),
                        mRearLeftModule.getPosition(),
                        mRearRightModule.getPosition(),
                        mFrontRightModule.getPosition(),
                },
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        // in CCW order from FL to FR
        mSwerveModules = new SwerveModule[] {
                mFrontLeftModule,
                mRearLeftModule,
                mRearRightModule,
                mFrontLeftModule
        };
    }

    /**
     * Resets the gyro
     */
    public void resetGyro() {
        Gyro.reset();
    }

    /**
     * Drives using cartesian speeds + a rotation speed
     * 
     * @param strafeXMetersPerSecond
     * @param forwardMetersPerSecond
     * @param rotationRadiansPerSecond
     * @param fieldRelative
     */
    public void driveFromCartesianSpeeds(double strafeXMetersPerSecond,
            double forwardMetersPerSecond,
            double rotationRadiansPerSecond,
            boolean fieldRelative) {
        ChassisSpeeds desiredChassisSpeeds;

        if (fieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(strafeXMetersPerSecond, forwardMetersPerSecond,
                    rotationRadiansPerSecond, Gyro.getRotation2d());
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(strafeXMetersPerSecond, forwardMetersPerSecond,
                    rotationRadiansPerSecond);
        }

        drive(desiredChassisSpeeds);
    }

    /**
     * Drives using an input ChassisSpeeds
     * 
     * @param desiredChassisSpeeds
     */
    public void drive(ChassisSpeeds desiredChassisSpeeds) {
        var swerveModuleStates = Kinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.kDriveMaxSpeedMetersPerSecond);

        drive(swerveModuleStates);
    }

    /**
     * Feeds the swerve modules each a desired state.
     * 
     * @param swerveModuleStates The new states of the modules in CCW order from FL
     *                           to FR
     */
    public void drive(SwerveModuleState[] swerveModuleStates) {
        mFrontLeftModule.setDesiredState(swerveModuleStates[0], _inHighGear);
        mRearLeftModule.setDesiredState(swerveModuleStates[1], _inHighGear);
        mRearRightModule.setDesiredState(swerveModuleStates[2], _inHighGear);
        mFrontRightModule.setDesiredState(swerveModuleStates[3], _inHighGear);
    }

    /**
     * Gets the current pose of the drivetrain from odometry
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Resets the position of odometry to the current position, minus 90
     */
    public void resetOdometry(Pose2d pose) {
        mSwerveModulePositions[0] = mFrontLeftModule.getPosition();
        mSwerveModulePositions[1] = mRearLeftModule.getPosition();
        mSwerveModulePositions[2] = mRearRightModule.getPosition();
        mSwerveModulePositions[3] = mFrontRightModule.getPosition();

        mOdometry.resetPosition(Gyro.getRotation2d().plus(Rotation2d.fromDegrees(-90)), mSwerveModulePositions, pose);
    }

    /**
     * Gets the direction the robot is facing in degrees, CCW+
     */
    public double getHeading() {
        return Gyro.getRotation2d().getDegrees();
    }

    /**
     * Sets the virtual gearbox shifter
     */
    public void setShift(boolean inHighGear) {
        _inHighGear = inHighGear;
    }

    /**
     * Toggles the virtual gearbox shifter
     */
    public void toggleShifter() {
        _inHighGear = !_inHighGear;
    }

    /**
     * Stops all drivetrain motors
     */
    public void stopMotors() {
        mFrontLeftModule.stopMotors();
        mRearLeftModule.stopMotors();
        mRearRightModule.stopMotors();
        mFrontRightModule.stopMotors();
    }

    /**
     * Sets the modules all to a single heading
     */
    public void setWheelAngles(Rotation2d angle) {
        mFrontLeftModule.setDesiredAngle(angle);
        mRearLeftModule.setDesiredAngle(angle);
        mRearRightModule.setDesiredAngle(angle);
        mFrontRightModule.setDesiredAngle(angle);
    }

    /**
     * Sets the modules to a open-loop speed
     * 
     * @param speedMetersPerSecond
     */
    public void setWheelSpeeds(double speedMetersPerSecond) {
        mFrontLeftModule.setDesiredSpeedOpenLoop(speedMetersPerSecond);
        mRearLeftModule.setDesiredSpeedOpenLoop(speedMetersPerSecond);
        mRearRightModule.setDesiredSpeedOpenLoop(speedMetersPerSecond);
        mFrontRightModule.setDesiredSpeedOpenLoop(speedMetersPerSecond);
    }

    /**
     * Sets the modules to a closed-loop velocity in MPS
     */
    public void setWheelVelocities(double speedMetersPerSecond) {
        mFrontLeftModule.setDesiredSpeed(speedMetersPerSecond, _inHighGear);
        mRearLeftModule.setDesiredSpeed(speedMetersPerSecond, _inHighGear);
        mRearRightModule.setDesiredSpeed(speedMetersPerSecond, _inHighGear);
        mFrontRightModule.setDesiredSpeed(speedMetersPerSecond, _inHighGear);
    }

    /**
     * Gets the module positions as an array ordered in standard CCW order
     * 
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                mFrontLeftModule.getPosition(),
                mRearLeftModule.getPosition(),
                mRearRightModule.getPosition(),
                mFrontRightModule.getPosition()
        };
    }

    /**
     * Updates odometry and any other periodic drivetrain events
     */
    @Override
    public void periodic() {
        // Update odometry
        var gyroAngle = Gyro.getRotation2d();
        var robotPose = mOdometry.update(gyroAngle, getModulePositions());
        mField.setRobotPose(robotPose);
    }
}
