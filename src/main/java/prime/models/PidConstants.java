package prime.models;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PidConstants implements Sendable {
    public double kP = 0.001;
    public double kI = 0;
    public double kD_min = 0;
    public double kF = 0.048;

    public PidConstants(double p) {
        kP = p;
        kI = 0;
        kD_min = 0;
        kF = 0;
    }

    public PidConstants(double p, double i) {
        kP = p;
        kI = i;
        kD_min = 0;
        kF = 0;
    }

    public PidConstants(double p, double i, double d) {
        kP = p;
        kI = i;
        kD_min = d;
        kF = 0;
    }

    public PidConstants(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD_min = d;
        kF = f;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kP", () -> kP, (newP) -> kP = newP);
        builder.addDoubleProperty("kI", () -> kI, (newI) -> kI = newI);
        builder.addDoubleProperty("kD", () -> kD_min, (newD) -> kD_min = newD);
        builder.addDoubleProperty("kF", () -> kF, (newF) -> kF = newF);
    }
}
