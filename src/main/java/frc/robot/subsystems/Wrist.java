package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

    public WPI_TalonFX wrist1;
    public WPI_TalonFX wrist2;

    public class Map {
        public static final int wrist1CanId = 0;
        public static final int wrist2CanId = 0;
        public static final double wristEjectCubeSpeed = -0.6;
    }

    public Wrist() {
        wrist1 = new WPI_TalonFX(Map.wrist1CanId);
        wrist2 = new WPI_TalonFX(Map.wrist2CanId);
    }

    public void ejectCube() {
        wrist1.set(ControlMode.PercentOutput, Map.wristEjectCubeSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

}
