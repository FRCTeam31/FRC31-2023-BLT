package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSidecar extends SubsystemBase {
    public enum LEDMode {
        OFF,
        AUTO_BLUE,
        AUTO_RED,
        TELEOP_BLUE,
        TELEOP_RED,
        SCAN_OUT,
        DISABLED_SCAN_UP_BLUE,
        DISABLED_SCAN_UP_RED,
        SCAN_DOWN
    }

    private SerialPort _arduinoSerial;
    private LEDMode _currentMode = LEDMode.OFF;
    private long _lastDisconnectionReportTimeMs = -1;

    public ArduinoSidecar() {
        try {
            _arduinoSerial = new SerialPort(115200, Port.kUSB);
        } catch (Exception ex) {
            DriverStation.reportError("[ERROR] Failed to initialize Arduino sidecar", false);
            DriverStation.reportError(ex.getMessage(), true);
        }
    }

    public void setMode(LEDMode mode) {
        if (_arduinoSerial != null) {
            _arduinoSerial.writeString(mode.toString());
            _arduinoSerial.flush();
        }
    }

    public LEDMode getMode() {
        return _currentMode;
    }

    @Override
    public void periodic() {
        try {
            if (_arduinoSerial.getBytesReceived() > 0) {
                var serialRead = _arduinoSerial.readString();
                System.out.print(">> [SIDECAR] ");
                System.out.println(serialRead);
            }
        } catch (Exception e) {
            var currentTimeMs = System.currentTimeMillis();
            var secondsSinceLastDiconnection = (currentTimeMs - _lastDisconnectionReportTimeMs) / 1000d;
            if (secondsSinceLastDiconnection > 5)
                DriverStation.reportError("[SIDECAR] Sidecar not connected", false);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addCloseable(_arduinoSerial);
        builder.addStringProperty("Current LED Mode",
                () -> _currentMode.toString(),
                (newMode) -> setMode(LEDMode.valueOf(newMode)));
    }
}
