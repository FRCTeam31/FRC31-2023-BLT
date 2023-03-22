package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CliffSensors extends SubsystemBase {
    private class CliffSensorsMap {
        public final static byte kArduinoAddress = 10;
        public final static byte kFrontLeftCornerIndex = 0;
        public final static byte kFrontRightCornerIndex = 1;
        public final static byte kRearLeftCornerIndex = 2;
        public final static byte kRearRightCornerIndex = 3;
    }

    public boolean IsFrontLeftCornerLevel = true;
    public boolean IsFrontRightCornerLevel = true;
    public boolean IsRearLeftCornerLevel = true;
    public boolean IsRearRightCornerLevel = true;

    private I2C i2c;

    public CliffSensors() {
        i2c = new I2C(Port.kOnboard, 1);
    }

    @Override
    public void periodic() {
        var buffer = new byte[32];
        if (!i2c.read(CliffSensorsMap.kArduinoAddress, 32, buffer))
            return; // Failed to get values from device

        var chars = new char[buffer.length];
        for (int i = 0; i < buffer.length; i++) {
            chars[i] = (char) buffer[i];
        }

        IsFrontLeftCornerLevel = chars[CliffSensorsMap.kFrontLeftCornerIndex] == '1';
        IsFrontRightCornerLevel = chars[CliffSensorsMap.kFrontRightCornerIndex] == '1';
        IsRearLeftCornerLevel = chars[CliffSensorsMap.kRearLeftCornerIndex] == '1';
        IsRearRightCornerLevel = chars[CliffSensorsMap.kRearRightCornerIndex] == '1';
    }
}
