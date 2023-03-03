package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.config.ShoulderMap;
import prime.movers.LazyCANSparkMax;

public class Shoulder extends PIDSubsystem {
    private LazyCANSparkMax shoulder1;
    private LazyCANSparkMax shoulder2;
    private WPI_CANCoder mEncoder;


    public Shoulder() {
        super(new PIDController(
            ShoulderMap.AnglePid.kP, 
            ShoulderMap.AnglePid.kI, 
            ShoulderMap.AnglePid.kD));

        shoulder1 = new LazyCANSparkMax(ShoulderMap.kShoulder1Id, MotorType.kBrushless);
        shoulder1.restoreFactoryDefaults();
        shoulder1.setIdleMode(IdleMode.kBrake);
        shoulder1.setOpenLoopRampRate(ShoulderMap.kOpenLoopRampRate);

        shoulder2 = new LazyCANSparkMax(ShoulderMap.kShoulder2Id, MotorType.kBrushless);
        shoulder1.restoreFactoryDefaults();
        shoulder2.follow(shoulder1);

        mEncoder = new WPI_CANCoder(ShoulderMap.kEncoderId);
    }

    public void runShoulder(double speed) {
        shoulder1.set(speed);
    }

    public void setShoulderAngle(double angleInDegrees) {
        setSetpoint(angleInDegrees);
    }

    @Override
    public double getMeasurement() {
        return mEncoder.getAbsolutePosition();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        
    }


    @Override
    public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Shoulder Position", this::getMeasurement, null);
    }

}
