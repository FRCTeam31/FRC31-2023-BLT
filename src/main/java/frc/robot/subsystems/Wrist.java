package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.movers.LazyCANSparkMax;
import prime.movers.LazySolenoid;
import frc.robot.config.WristMap;

public class Wrist extends SubsystemBase {
    private CANSparkMax mWristMotor;
    private LazySolenoid mOutActuator;
    private LazySolenoid mInActuator;
    private Compressor compressor;

    public Wrist() {
        mWristMotor = new CANSparkMax(WristMap.kWrist1Id, MotorType.kBrushless);
        mWristMotor.restoreFactoryDefaults();
        mWristMotor.setOpenLoopRampRate(0.2);

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        mOutActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId);
        mInActuator = new LazySolenoid(PneumaticsModuleType.CTREPCM, WristMap.kWristActuatorId + 1);
        setWrist(false);
    }

    public void stopIntake() {
        mWristMotor.stopMotor();

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
        mWristMotor.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Actuated", this::getWristOut, this::setWrist);
        builder.addBooleanProperty("Pressure switch", compressor::getPressureSwitchValue, null);
    }

}
