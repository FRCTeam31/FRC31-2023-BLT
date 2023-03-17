package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;
import prime.movers.LazySolenoid;
import frc.robot.config.WristMap;

public class Wrist extends SubsystemBase {
    private LazyCANSparkMax wrist1;
    private LazySolenoid mOutActuator;
    private LazySolenoid mInActuator;
    private Compressor compressor;
    private DigitalInput mHallSensor;

    public Wrist() {
        wrist1 = new LazyCANSparkMax(WristMap.kWrist1Id, MotorType.kBrushless);
        wrist1.restoreFactoryDefaults();
        wrist1.setOpenLoopRampRate(0.5);

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        mOutActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId);
        mInActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId + 1);
        setWrist(false);

        mHallSensor = new DigitalInput(WristMap.kWristHallSensorChannel);
    }

    public void stopIntake() {
        wrist1.stopMotor();

    }

    public void toggleWrist() {
        mOutActuator.toggle();
        mInActuator.toggle();
    }

    public void setWrist(boolean out) {
        mOutActuator.set(out);
        mInActuator.set(!out);
    }

    public boolean getWristOut() {
        return mOutActuator.get();
    }

    public void runMotors(double speed) {
        wrist1.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Hall Sensor", () -> !mHallSensor.get(), null);
        builder.addBooleanProperty("Actuated", this::getWristOut, this::setWrist);
        builder.addBooleanProperty("Pressure switch", compressor::getPressureSwitchValue, null);
    }

}
