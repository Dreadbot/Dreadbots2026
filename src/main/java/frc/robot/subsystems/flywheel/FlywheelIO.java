package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double RPM = 0.0;
        public double appliedVolts = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
