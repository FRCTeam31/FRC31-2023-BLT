package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSidecar extends SubsystemBase {
    private class Map {
        private static final int ArduinoAddress = 10;
    }

    public enum StripModes {
        OFF,
        SOLID_GREEN,
        GIMMEACONE,
        GIMMEACUBE,
        MODE5,
        MODE6,
        MODE7,
        MODE8,
        MODE9,
    }

    public enum LEDStrips {
        FRONT,
        INDICATOR,
        ACCENT_LEFT,
        ACCENT_RIGHT
    }

    private I2C mArduinoI2c;
    private byte[] mCurrentModes = new byte[] { 0, 0, 0, 0 };

    public ArduinoSidecar() {
        try {
            mArduinoI2c = new I2C(Port.kOnboard, Map.ArduinoAddress);
        } catch (Exception ex) {
            DriverStation.reportError("[ERROR] Failed to initialize ArduinoSidecar", true);
        }
    }

    public void setLEDStripMode(LEDStrips strip, StripModes mode) {
        var bytesToSend = new byte[mCurrentModes.length];
        for (int i = 0; i < bytesToSend.length; i++) {
            bytesToSend[i] = (byte) mCurrentModes[i];
        }

        if (!mArduinoI2c.writeBulk(bytesToSend, bytesToSend.length))
            DriverStation.reportError("[ERROR] Aborted while sending LED update to ArduinoSidecar", true);
    }

    public boolean[] getCliffSensorStates() {
        var readBytes = new byte[4];
        if (!mArduinoI2c.read(0, 4, readBytes))
            return new boolean[] { false, false, false, false };

        var result = new boolean[4];
        for (int i = 0; i < readBytes.length; i++) {
            result[i] = readBytes[i] == 1;
        }

        return result;
    }
}
