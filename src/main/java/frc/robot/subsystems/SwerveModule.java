package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.config.DriveMap;
import prime.utilities.CTREConverter;
import prime.movers.LazyWPITalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SwerveModule extends PIDSubsystem {
   private LazyWPITalonFX mSteeringMotor;
   private LazyWPITalonFX mDriveMotor;
   private WPI_CANCoder mEncoder;
   private int mEncoderOffset;

   public SwerveModule(
         int driveMotorId,
         int steeringMotorId,
         int encoderId,
         int encoderAbsoluteOffset,
         boolean driveInverted) {
      super(new PIDController(DriveMap.kSteeringPidConstants.kP, DriveMap.kSteeringPidConstants.kI,
            DriveMap.kSteeringPidConstants.kD));
      mEncoderOffset = encoderAbsoluteOffset;

      // Set up the steering motor
      mSteeringMotor = new LazyWPITalonFX(steeringMotorId);
      mSteeringMotor.configFactoryDefault();
      mSteeringMotor.clearStickyFaults();
      mSteeringMotor.setNeutralMode(NeutralMode.Brake);
      mSteeringMotor.setInverted(TalonFXInvertType.CounterClockwise);
      mSteeringMotor.configOpenloopRamp(0.2);

      // Set up the drive motor
      mDriveMotor = new LazyWPITalonFX(driveMotorId);

      mDriveMotor.configFactoryDefault();
      mDriveMotor.clearStickyFaults();
      mDriveMotor.setNeutralMode(NeutralMode.Brake);
      mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the Falcon is the falcon's encoder
      mDriveMotor.configOpenloopRamp(0.2);
      mDriveMotor.setInverted(driveInverted ? TalonFXInvertType.CounterClockwise
         : TalonFXInvertType.Clockwise);

      // Set up our encoder
      mEncoder = new WPI_CANCoder(encoderId);
      mEncoder.clearStickyFaults();
      mEncoder.configFactoryDefault();
      mEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

      // Create a PID controller to calculate steering motor output
      TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
      driveMotorConfig.slot0.kP = DriveMap.kDrivePidConstants.kP;
      mDriveMotor.configAllSettings(driveMotorConfig);

      getController().enableContinuousInput(-Math.PI, Math.PI);
      getController().setTolerance(Math.PI / 180);
      enable();
   }

   @Override
   public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Encoder position", this::getEncoderPosition, this::setEncoderPosition);
      builder.addDoubleProperty("Encoder absolute position", this::getEncoderAbsolutePosition, this::setEncoderPosition);
   }

   public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
            CTREConverter.falconToMeters(
                  mDriveMotor.getSelectedSensorPosition(),
                  DriveMap.kDriveWheelCircumference,
                  DriveMap.kDriveGearRatio),
            getAbsoluteRotation2d());
   }

   public void setDesiredAngle(Rotation2d angle) {
      getController().setSetpoint(MathUtil.angleModulus(angle.getRadians()));
   }

   public void setDesiredSpeed(double speedMetersPerSecond) {
      var desiredVelocity20ms = (speedMetersPerSecond / 50) * DriveMap.falconTotalSensorUnits;
      var desiredRotationsPer20ms = desiredVelocity20ms / DriveMap.kDriveWheelCircumference;
      var desiredVelocity = (desiredRotationsPer20ms * DriveMap.falconTotalSensorUnits * 5);
      mDriveMotor.set(ControlMode.Velocity, desiredVelocity);
   }

   /**
    * Sets the desired state of the module.
    * 
    * @param desiredState The state of the module that we'd like to be at in this
    *                     period
    */
   public void setDesiredState(SwerveModuleState desiredState) {
      // Optimize the state to avoid turning wheels further than 90 degrees
      var encoderRotation = getAbsoluteRotation2d();
      desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);

      setDesiredSpeed(desiredState.speedMetersPerSecond);
      setDesiredAngle(desiredState.angle);
   }

   public void setEncoderPosition(double newPosition) {
      mEncoder.setPosition(newPosition);
   }

   public double getEncoderPosition() {
      return mEncoder.getPosition();
   }

   public void setEncoderPositionToAbsolute() {
      mEncoder.setPositionToAbsolute();
   }

   public double getEncoderAbsolutePosition() {
      return mEncoder.getAbsolutePosition();
   }

   public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(mEncoder.getPosition());
   }

   @Override
   protected void useOutput(double output, double setpoint) {
      mSteeringMotor.set(ControlMode.PercentOutput, MathUtil.clamp(output, -1, 1));
   }

   @Override
   protected double getMeasurement() {
      var currentPositionRadians = getAbsoluteRotation2d().getRadians();
      return currentPositionRadians;
   }

   private Rotation2d getAbsoluteRotation2d() {
      return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition() - mEncoderOffset);
   }
}
